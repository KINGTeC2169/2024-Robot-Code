package frc.robot.subsystems;


import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
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

public class Arm extends SubsystemBase {

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    TalonFX leftArm = new TalonFX(Constants.DeviceID.leftArm);
    TalonFX rightArm = new TalonFX(Constants.DeviceID.rightArm);

    //Master = left, follower = right
    private final DutyCycleEncoder followEncoder = new DutyCycleEncoder(Constants.Ports.rightArmEncoder);
    private final DutyCycleEncoder masterEncoder = new DutyCycleEncoder(Constants.Ports.leftArmEncoder);
    //Update hex encoder

    final PositionDutyCycle request = new PositionDutyCycle(0);
    PIDController angleLoop = new PIDController(0.5,0,0);
    private GenericEntry angleP = tab.add("angleP", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).getEntry();
    private GenericEntry angleI = tab.add("angleI", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).getEntry();
    private GenericEntry angleD = tab.add("angleD", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).getEntry();

    double setAngle;
    double difference;

    //Degrees to rotations
    final double oneRotation = Constants.Motors.armGearBox;

    final double zero = 0.04; //zero angle
    final double armLowerLimit = 0.96;
    final double armUpperLimit = 0.66; //Whatever is need for amp

    
    
    public Arm() {

        var configs = new Slot0Configs();
        configs.kP = 0.5;
        
        leftArm.getConfigurator().apply(configs,0.050);
        rightArm.getConfigurator().apply(configs,0.050);
        
        difference = getAngle()-getFollowAngle();
        
        tab.addDouble("Master Angle", () -> getAngle());
        

        tab.addBoolean("Arm Ready", () -> armReady());
        tab.addBoolean("Arm On", () -> !off());
        
        tab.addDouble("Voltage Left", () -> getArmVoltage()[0]);
        tab.addDouble("Voltage Right", () -> getArmVoltage()[1]);
        tab.addDouble("Current Left", () -> getArmCurrent()[0]);
        tab.addDouble("Current Right", () -> getArmCurrent()[1]);



    }

    public void setSpeed(double speed){
        leftArm.set(speed * 0.25);
        rightArm.set(-speed * 0.25);
    }         

    public double getAngle(){
        return 1 - masterEncoder.getAbsolutePosition();
    }

    public double getFollowAngle(){
        return followEncoder.getAbsolutePosition();
    }

    //Used to correct the follow motor
    public double getDifference(){
        return (getAngle() - getFollowAngle()) - difference;
    }

    //Rotations
    public void setAngle(double angle) {
        setAngle = angle;

        leftArm.setControl(request.withPosition(angleLoop.calculate(getAngle(), angle)/12));
        rightArm.setControl(request.withPosition(angleLoop.calculate(getAngle(), angle)/12));
    }

    public boolean armReady(){
        return setAngle == getAngle();
    }


    public void armStop(){
        // double stopLeftPos = leftArmEncoder.get();
        // double stopRightPos = rightArmEncoder.get();
        // leftArm.set(angleLoop.calculate(stopLeftPos, leftArm.getPosition().getValue()));
        // rightArm.set(angleLoop.calculate(stopRightPos, rightArm.getPosition().getValue()));
        leftArm.set(0);
        rightArm.set(0);
    }

    //Extra:
    public boolean off(){

        return zero == getAngle(); //Todo: find rest voltage
    }

    public double[] getArmCurrent(){
        return new double[]{leftArm.getSupplyCurrent().getValueAsDouble(),
                            rightArm.getSupplyCurrent().getValueAsDouble()}; 
    }

    public double[] getArmVoltage(){
        return new double[]{leftArm.getSupplyVoltage().getValueAsDouble(),
                            rightArm.getSupplyVoltage().getValueAsDouble()}; 
    }

    
}
