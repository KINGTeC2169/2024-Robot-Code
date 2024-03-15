package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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

    private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Ports.armEncoder);

    private PIDController armPID;
    private ArmFeedforward armForward;

    private double setPosition;
    private final double lowerLimit = Positions.rest; 
    private final double upperLimit = 0.435;

    
    public Arm() {

        var configs = new Slot0Configs();
        configs.kP = 0.5;
        
        leftArm.getConfigurator().apply(configs);
        rightArm.getConfigurator().apply(configs);

        encoder.setPositionOffset(Constants.ArmConstants.armEncoderOffset);
        
        //(2.35, 0.075, 0)
        //4.25
        armPID = new PIDController(26, 0.05, 5);
        armForward = new ArmFeedforward(0.15, 0.22, 3.61, 0.01);
  
        tab.add("Arm PID", armPID).withSize(2, 2).withPosition(0, 0);

        ShuffleboardLayout leftMotor = tab.getLayout("Left Motor", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        leftMotor.addDouble("Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        leftMotor.addDouble("Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0).withProperties(Map.of("Max", 5));

        ShuffleboardLayout rightMotor = tab.getLayout("Right Motor", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        rightMotor.addDouble("Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        rightMotor.addDouble("Current", () -> getCurrent()[1]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0).withProperties(Map.of("Max", 5));

        //tab.addDouble("Encoder Position", () -> getPosition()).withWidget(BuiltInWidgets.kDial).withSize(2, 2).withProperties(Map.of("Max", 0.75, "Min", 0.25)).withPosition(0, 2);
        tab.addDouble("Encoder Position", () -> getPosition()).withSize(1, 1).withPosition(6, 1);
        tab.addDouble("Encoder Graph", () -> getPosition()).withWidget(BuiltInWidgets.kGraph).withSize(3, 3).withPosition(7, 1);
        tab.addDouble("Abs Encoder Position", () -> encoder.getAbsolutePosition()).withPosition(7, 0); //Don't change right now
        tab.addDouble("Aim", () -> armToAim(getPosition())).withPosition(6, 0);
        tab.addBoolean("Arm Ready", () -> isReady()).withPosition(6, 2).withSize(1, 2);
        tab.addDouble("Converted", () -> aimToArm(armToAim(getPosition())));
    }

    public void setSpeed(double speed){
        leftArm.set(speed);
        rightArm.set(speed);
    }         

    public double getPosition(){
        double pos = (encoder.getPositionOffset() - encoder.getAbsolutePosition());
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

    public void setRest(){
        setPosition = Positions.rest;
        setSpeed(-0.08);
    }

    public void setAmp(){
        setPosition = Positions.amp;
        leftArm.setVoltage(armPID.calculate(getPosition(), Positions.amp));
        rightArm.setVoltage(armPID.calculate(getPosition(),  Positions.amp));
    }

    public void setShootPos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        setPosition = position;
        leftArm.setVoltage(armPID.calculate(getPosition(), position) + armForward.calculate(position - 0.25, 0));
        rightArm.setVoltage(armPID.calculate(getPosition(),  position) + armForward.calculate(position - 0.25, 0));
    }

    //AIM = DEGREES, ARM = ROTATIONS
    /*public void setAim(double aim) {
        setShootPos(aimToArm(aim));
    }*/

    public void activeStop(){
        leftArm.setVoltage(armForward.calculate(getPosition()-0.25, 0));
        rightArm.setVoltage(armForward.calculate(getPosition()-0.25, 0));
    }

    public void stop(){
        leftArm.set(0);
        rightArm.set(0);
    }

    public boolean isReady(){
        if ( Math.abs(setPosition-getPosition()) < 0.005){
            LEDs.green();
            return true;
            
        }else{
            LEDs.red();
            return false; 
        }
    }

    public boolean restReady(){
        return getPosition() <= Positions.rest;
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

    //Input aim DEGREES instead of position ROTATIONS
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
        return -(aim - ArmConstants.restAim)/360 + Positions.rest;
    }

    /**
     * Converts aim to position of the arm
     * @param position position of the arm IN ROTATIONS
     * @return aim angle IN DEGREES
     */
    public static double armToAim(double position){
        return ArmConstants.restAim - ((position-Positions.rest)*360);
    }
}
