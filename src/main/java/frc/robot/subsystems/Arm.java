package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Positions;
import frc.robot.Constants.Vision;

public class Arm extends SubsystemBase {

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    TalonFX leftArm = new TalonFX(Constants.Ports.leftArm);
    TalonFX rightArm = new TalonFX(Constants.Ports.rightArm);

    //Master = left, follower = right
    private  DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Ports.armEncoder);
    //Update hex encoder

    private PIDController armPID;

    private double setPosition;
    final double zero = 0.04; //zero angle   
    
    public Arm() {

        var configs = new Slot0Configs();
        configs.kP = 0.5;
        
        leftArm.getConfigurator().apply(configs);
        rightArm.getConfigurator().apply(configs);

        encoder.setPositionOffset(Constants.ArmConstants.armEncoderOffset);
        
        armPID = new PIDController(1.95, 0.075, 0);

        tab.add("Arm PID", armPID).withSize(2, 2).withPosition(0, 0);

        ShuffleboardLayout leftMotor = tab.getLayout("Left Motor", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        leftMotor.addDouble("Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        leftMotor.addDouble("Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0);

        ShuffleboardLayout rightMotor = tab.getLayout("Right Motor", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        rightMotor.addDouble("Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        rightMotor.addDouble("Current", () -> getCurrent()[1]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0);

        tab.addDouble("Encoder Position", () -> getPosition()).withWidget(BuiltInWidgets.kDial).withSize(2, 2).withProperties(Map.of("Max", 0.75, "Min", 0.25)).withPosition(0, 2);
        //tab.addDouble("Encoder Position", () -> getPosition()).withSize(1, 1).withPosition(6, 1);
        tab.addDouble("Encoder Graph", () -> getPosition()).withWidget(BuiltInWidgets.kGraph).withSize(3, 3).withPosition(7, 1);
        tab.addDouble("Abs Encoder Position", () -> encoder.getAbsolutePosition()).withPosition(7, 0); //Don't change right now
        tab.addDouble("Aim", () -> armToAim(getPosition())).withPosition(6, 0);
        //tab.addDouble("Converted Position", () -> aimToArm(armToAim(getPosition())));
        tab.addBoolean("Arm Ready", () -> isReady()).withPosition(6, 2).withSize(1, 2);
    
    }

    public void setSpeed(double speed){
        leftArm.set(speed);
        rightArm.set(speed);
    }         

    public double getPosition(){
        double pos = (encoder.getPositionOffset() - encoder.getAbsolutePosition());
        //double pos = (1-encoder.getAbsolutePosition()) - encoder.getPositionOffset();
        if(pos < 0){
            pos += 1;
            Math.abs(pos);
        }
        return pos;
    }

    public double[] getCurrent(){
        return new double[]{leftArm.getSupplyCurrent().getValueAsDouble(),
                            rightArm.getSupplyCurrent().getValueAsDouble()}; 
    }

    public double[] getVoltage(){
        return new double[]{leftArm.getSupplyVoltage().getValueAsDouble(),
                            rightArm.getSupplyVoltage().getValueAsDouble()}; 
    }

    public void setPosition(double position) {
        /*
        if(position == Positions.rest){
            System.out.println("!!!");
            if(getPosition() > 0.4){
                leftArm.set(-0.15);
                rightArm.set(-0.15);
            } else {
                leftArm.set(armDownPID.calculate(getPosition(), position));
                rightArm.set(armDownPID.calculate(getPosition(),  position));
            }
            return;
        }*/
        setPosition = position;
        if (position > 0.40 && !(position == Positions.amp)){
            position = 0.40;
            setPosition = position;
        }else if (position < 0.292){
            position = 0.292;
            setPosition = position;
        }
        leftArm.set(armPID.calculate(getPosition(), position));
        rightArm.set(armPID.calculate(getPosition(),  position));
    }

    //AIM = DEGREES, ARM = ROTATIONS
    public void setAim(double aim) {
        double position = aimToArm(aim);
        setPosition(position);
    }

    public void activeStop(){
        if(setPosition == Positions.amp){
            
        }else if (setPosition > 0.40){
            setPosition = 0.40;
        }else if (setPosition <= 0.292){
            setPosition = 0.292;
        }
        leftArm.set(armPID.calculate(getPosition(), setPosition));
        rightArm.set(armPID.calculate(getPosition(), setPosition));
    }

    //Use activeStop() intead!
    public void stop(){
        leftArm.set(0);
        rightArm.set(0);
    }

    public boolean isReady(){
        return Math.abs(setPosition-getPosition()) < 0.01;
    }

    public boolean autoReady() {
        return Math.abs(setPosition-getPosition()) < 0.015;
    }


    /**
     * Gets the arm position in space (used for limelight) NOT encoder position
     * @return x,y distance compared to the point of rotation 
     */
    public double[] getArmPosition(){
        double x = ArmConstants.distance*Math.cos(2*Math.PI*(getPosition()-0.25) + ArmConstants.armOffset) + Vision.toShaftX;
        double y = ArmConstants.distance*Math.cos(2*Math.PI*(getPosition()-0.25) + ArmConstants.armOffset) + Vision.toShaftY;
        return new double[]{x,y};
    }

    /**
     * Gets the arm position in space (used for limelight) NOT encoder position
     * @param position position of the arm
     * @return x,y distance compared to the point of rotation 
     */
    public static double[] predictArmPosition(double position){
        double x = ArmConstants.distance*Math.cos(2*Math.PI*(position-0.25) + ArmConstants.armOffset) + Vision.toShaftX;
        double y = ArmConstants.distance*Math.cos(2*Math.PI*(position-0.25) + ArmConstants.armOffset) + Vision.toShaftY;
        return new double[]{x,y};
    }

    public static double[] predictArmPositionAim(double aim){
        double position = aimToArm(aim);
        double x = ArmConstants.distance*Math.cos(2*Math.PI*(position-0.25) + ArmConstants.armOffset) + Vision.toShaftX;
        double y = ArmConstants.distance*Math.cos(2*Math.PI*(position-0.25) + ArmConstants.armOffset) + Vision.toShaftY;
        return new double[]{x,y};
    }

    /**
     * Converts aim to position of the arm
     * @param aim angle IN DEGREES we want to aim
     * @return position of the arm IN ROTATIONS
     */
    public static double aimToArm(double aim){

        aim *= 0.0174533;

        double slope1 = Math.tan(-aim);
        double totalOffset = Math.atan(-1/slope1);
        double encoderRad = totalOffset - ArmConstants.shooterOffset; //Radians  - ArmConstants.armOffset
        double encoderRot = (encoderRad/(2*Math.PI) + 0.25); //Rotations
            
        if(encoderRot < 0) encoderRot += 0.5;
            
        return encoderRot;
    }

    /**
     * Converts aim to position of the arm
     * @param position position of the arm IN ROTATIONS
     * @return aim angle IN DEGREES
     */
    public static double armToAim(double position){
        double encoderRad = 2*Math.PI*(position-0.25);

        double slope = -1/Math.tan(encoderRad + ArmConstants.shooterOffset);//ArmConstants.armOffset

        double aim = -(Math.atan(slope) * (180/Math.PI));

        return aim;
    }
}
