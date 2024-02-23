package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
        
        armPID = new PIDController(0.5, 0, 0);

        tab.add("Arm PID", armPID).withSize(2, 2).withPosition(0, 0);

        ShuffleboardLayout leftMotor = tab.getLayout("Left Motor", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        leftMotor.addDouble("Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        leftMotor.addDouble("Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0);

        ShuffleboardLayout rightMotor = tab.getLayout("Right Motor", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        rightMotor.addDouble("Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withPosition(0, 1).withProperties(Map.of("Max", 12));
        rightMotor.addDouble("Current", () -> getCurrent()[1]).withWidget(BuiltInWidgets.kDial).withPosition(0, 0);

        tab.addDouble("Encoder Position", () -> getPosition()).withWidget(BuiltInWidgets.kDial).withSize(2, 2).withProperties(Map.of("Max", 1, "Min", -1)).withPosition(0, 2);
    }

    public void setSpeed(double speed){
        leftArm.set(speed * 0.25);
        rightArm.set(speed * 0.25);
    }         

    public double getPosition(){
        return encoder.getAbsolutePosition();
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
        setPosition = position;
        if (position > 0.5){
            position = 0;
        }
        leftArm.set(armPID.calculate(getPosition(), position));
        rightArm.set(-armPID.calculate(getPosition(),  position));
    }

    public void activeStop(){
        leftArm.set(armPID.calculate(getPosition(), setPosition));
        rightArm.set(armPID.calculate(getPosition(), setPosition));
    }

    //Use activeStop() intead!
    public void stop(){
        leftArm.set(0);
        rightArm.set(0);
    }

    public boolean isReady(){
        return setPosition == getPosition();
    }
}
