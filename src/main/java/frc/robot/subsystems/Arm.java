package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
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
    private final double upperLimit = Positions.amp+0.05;

    //0.34
    private GenericEntry adShoot = tab.add("adjust Shoot", 0).withPosition(1,0).getEntry();

    public Arm() {

        encoder.setPositionOffset(Constants.ArmConstants.armEncoderOffset);
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0;

        leftArm.getConfigurator().apply(talonFXConfigs);
        rightArm.getConfigurator().apply(talonFXConfigs);
        leftArm.setNeutralMode(NeutralModeValue.Brake);
        rightArm.setNeutralMode(NeutralModeValue.Brake);

        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));//true means inverse

        armPID = new PIDController(55,0.7,1);
        armForward = new ArmFeedforward(0.15, 0.22, 3.61, 0.01);
    
        ShuffleboardLayout leftMotor = tab.getLayout("Left Motor", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 2).withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
        leftMotor.addDouble("Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        leftMotor.addDouble("Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0).withProperties(Map.of("Max", 5));

        ShuffleboardLayout rightMotor = tab.getLayout("Right Motor", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 2).withProperties(Map.of("Number of columns", 1, "Number of rows", 2));
        rightMotor.addDouble("Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        rightMotor.addDouble("Current", () -> getCurrent()[1]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0).withProperties(Map.of("Max", 5));

        //tab.addDouble("Encoder Position", () -> getPosition()).withWidget(BuiltInWidgets.kDial).withSize(2, 2).withProperties(Map.of("Max", 0.75, "Min", 0.25)).withPosition(0, 2);
        tab.addDouble("Encoder Position", () -> getPosition()).withSize(1, 1).withPosition(6, 1);
        tab.addDouble("MM Graph", () -> leftArm.getPosition().getValueAsDouble()).withWidget(BuiltInWidgets.kGraph).withSize(3, 3).withPosition(7, 1);
        tab.addDouble("Abs Encoder Position", () -> encoder.getAbsolutePosition()).withPosition(7, 0); //Don't change right now
        tab.addDouble("Aim", () -> armToAim(getPosition())).withPosition(6, 0);
        tab.addBoolean("Arm Ready", () -> isReady()).withPosition(6, 2).withSize(1, 2);
        tab.addDouble("Converted", () -> aimToArm(armToAim(getPosition())));
        tab.addDouble("MM Position", () -> leftArm.getPosition().getValueAsDouble());
    }

    /**Part of Music subsystem commands*/
    public void playNote(double hz){
        leftArm.setControl(new MusicTone(hz));
        rightArm.setControl(new MusicTone(hz));
    }
    
    /**Gets the arm the last position it was set to*/
    public double getSetPosition(){
        return setPosition;
    }

    /**Sets the arm voltage*/
    public void setVoltage(double volts){
        leftArm.setVoltage(volts);
    }

    /**Gets the position of the arm from the hex encoder */
    public double getPosition(){
        double pos = (encoder.getPositionOffset() - encoder.getAbsolutePosition());
        if(pos < 0){
            pos += 1;
            Math.abs(pos);
        }
        return pos;
    }

    /**Returns the output current of the left and right motors */
    public double[] getCurrent(){
        return new double[]{leftArm.getSupplyCurrent().getValueAsDouble(),
                            rightArm.getSupplyCurrent().getValueAsDouble()}; 
    }

    /**Returns the input voltage of the left and right motors */
    public double[] getVoltage(){
        return new double[]{leftArm.getSupplyVoltage().getValueAsDouble(),
                            rightArm.getSupplyVoltage().getValueAsDouble()}; 
    }

    /**Sets the arm to rest postion.
     * 
     * @param ampMode - arm will move faster if true
     */
    public void setRest(boolean ampMode){
        setPosition = Positions.rest;
        if (ampMode) this.setVoltage(-3.5);
        else this.setVoltage(-2);
    }

    /**Sets the arm to the amp scoring position */
    public void setAmp(){
        setPosition = Positions.amp;
        this.setShootPos(Positions.amp);
    }


    public void setShootPos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        setPosition = position;
        leftArm.setVoltage(armPID.calculate(getPosition(), position) + armForward.calculate(position - 0.25, 0));
    }

    public void superShoot(){
        setPosition = MathUtil.clamp(adShoot.getDouble(0), lowerLimit, upperLimit);
        leftArm.setVoltage(armPID.calculate(getPosition(), setPosition) + armForward.calculate(setPosition - 0.25, 0));
    }

    //AIM = DEGREES, ARM = ROTATIONS
    /*public void setAim(double aim) {
        setShootPos(aimToArm(aim));
    }*/

    /**Sets the arm to its last set position */
    public void activeStop(){
        leftArm.setPosition(setPosition);
    }

    /**Stops the left and right motors */
    public void stop(){
        leftArm.set(0);
        rightArm.set(0);
    }

    /**Returns true if the position of the arm is +/- 0.003 of the set position and the arm is not in the rest position.
     * LEDs are also synced to this method.
     */
    public boolean isReady(){
        if (Math.abs(setPosition-getPosition()) < 0.003 && !restReady()){
            LEDs.green();
            return true;
            
        }else{
            LEDs.red();
            return false; 
        }
    }

    /**Returns true if the arm is in rest position */
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
