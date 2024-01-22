package frc.robot.subsystems;


import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
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

public class Arm extends SubsystemBase {
    TalonFX leftArm = new TalonFX(Constants.DeviceID.leftArm);
    TalonFX rightArm = new TalonFX(Constants.DeviceID.rightArm);

    DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.DeviceID.armEncoder);
    final PositionDutyCycle request = new PositionDutyCycle(0);
    PIDController angleLoop = new PIDController(0,0,0);

    final double armLowerLimit = 0;
    final double armUpperLimit = 90;

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    public Arm() {
        
        ShuffleboardLayout currents = tab.getLayout("Arm Currents", BuiltInLayouts.kGrid).withSize(2, 1).withProperties(Map.of("Number of rows", 1)).withPosition(0, 0);

        var configs = new Slot0Configs();
        configs.kP = 0.1;
        
        leftArm.getConfigurator().apply(configs,0.050);
        rightArm.getConfigurator().apply(configs,0.050);
        
        tab.addDouble("Absolute Angle", () -> getAngle()).withPosition(7, 2);
        currents.addDouble("Left Arm Current", () -> getArmCurrent()[0]).withWidget(BuiltInWidgets.kVoltageView);
        currents.addDouble("Right Arm Current", () -> getArmCurrent()[1]).withWidget(BuiltInWidgets.kVoltageView);

    }

    public double getAngle() {
        return (armEncoder.getAbsolutePosition() - armEncoder.getPositionOffset()) * 360;
    }


    public void setAngle(double angle) {
        leftArm.setControl(request.withPosition(angleLoop.calculate(getAngle(), angle)/12));
        rightArm.setControl(request.withPosition(angleLoop.calculate(getAngle(), angle)/12));
    }

    public void armStop(){
        leftArm.set(0);
        rightArm.set(0);
    }

    public double[] getArmCurrent(){
        return new double[]{leftArm.getSupplyCurrent().getValueAsDouble(),
                            rightArm.getSupplyCurrent().getValueAsDouble()};  
    }
}
