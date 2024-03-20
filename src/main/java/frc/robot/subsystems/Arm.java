package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

import com.ctre.phoenix.music.Orchestra;

public class Arm extends SubsystemBase {

    private final PositionVoltage m_position = new PositionVoltage(0);

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    TalonFX leftArm = new TalonFX(Constants.Ports.leftArm);
    TalonFX rightArm = new TalonFX(Constants.Ports.rightArm);

    private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Ports.armEncoder);

    private PIDController armPID;
    private ArmFeedforward armForward;

    private double setPosition;
    private double talonZero;
    private final double lowerLimit = Positions.rest; 
    private final double upperLimit = Positions.amp+0.05;

    private GenericEntry kS = tab.add("kS", 0.11).withPosition(1,0).getEntry();//Done
    private GenericEntry kG = tab.add("kG", 0).withPosition(1,1).getEntry();
    private GenericEntry kV = tab.add("kV", 0).withPosition(1,2).getEntry();
    private GenericEntry kA = tab.add("kA", 0).withPosition(1,3).getEntry();
    private GenericEntry kP = tab.add("kP", 0).withPosition(0,0).getEntry();
    private GenericEntry kI = tab.add("kI", 0).withPosition(0,1).getEntry();
    private GenericEntry kD = tab.add("kD", 0).withPosition(0,2).getEntry();

    public Arm() {

        encoder.setPositionOffset(Constants.ArmConstants.armEncoderOffset);
        resetArmEncoder();

        updatePID();
        

        armPID = new PIDController(55,0.7,1);
        //armForward = new ArmFeedforward(0.15, 0.22, 3.61, 0.01);
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

    public void resetArmEncoder() {
        if(encoder.isConnected()){
            talonZero = getPosition();
        }
    }


    public void updatePIDF(){
        //armPID = new PIDController(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0));
        //armForward = new ArmFeedforward(kS.getDouble(0.18), kG.getDouble(0.41), kV.getDouble(3.88), kA.getDouble(0.03));
        System.out.println("armPID reset");
    }
    //Update PID and feedforward with values inputted on shuffleboard
    public void updatePID(){
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        System.out.println(kP.getDouble(0));
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kS.getDouble(0.18); //0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kG = kG.getDouble(0.41);
        slot0Configs.kV = kV.getDouble(3.88); //0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = kA.getDouble(0.03); //0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = kP.getDouble(0); // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = kI.getDouble(0); // no output for integrated error
        slot0Configs.kD = kD.getDouble(0); // no output for error derivative

        leftArm.getConfigurator().apply(talonFXConfigs);
        rightArm.getConfigurator().apply(talonFXConfigs);

        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));//true means inverse

    }

    public void playNote(double hz){
        leftArm.setControl(new MusicTone(hz));
        //rightArm.setControl(new MusicTone(hz));
    }
    
    public void setVoltage(double volts){
        leftArm.setVoltage(volts);
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
        this.setVoltage(-2.5);
    }

    public void posOne(){
        leftArm.setControl(m_position.withPosition(10-talonZero));
    }

    public void posZero(){
        leftArm.setControl(m_position.withPosition(0-talonZero));
    }

    public void setAmp(){
        setPosition = Positions.amp;
        leftArm.setPosition(Positions.amp);
    }

    /*public void setShootPos(double position){
        leftArm.setControl(m_position.withPosition((0-talonZero)*216));
    }*/

    public void setShootPos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        setPosition = position;
        //System.out.println(armForward.calculate(position - 0.25, 0));
        //System.out.println(armPID.calculate(getPosition(), position) + " PID");
        leftArm.setVoltage(armPID.calculate(getPosition(), position) + armForward.calculate(position - 0.25, 0));
    }

    //AIM = DEGREES, ARM = ROTATIONS
    /*public void setAim(double aim) {
        setShootPos(aimToArm(aim));
    }*/

    public void activeStop(){
        leftArm.setPosition(setPosition);
    }

    public void stop(){
        leftArm.set(0);
        rightArm.set(0);
    }

    public boolean isReady(){
        if (Math.abs(setPosition-getPosition()) < 0.003 && !restReady()){
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
