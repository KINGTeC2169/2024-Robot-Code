package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    TalonFX leftArm = new TalonFX(Constants.DeviceID.leftArm);
    TalonFX rightArm = new TalonFX(Constants.DeviceID.rightArm);

    DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.DeviceID.armEncoder);
    final VoltageOut request = new VoltageOut(0);
    PIDController angleLoop = new PIDController(0,0,0);

    final double armLowerLimit = 0;
    final double armUpperLimit = 90;

    public double getAngle() {
        return (armEncoder.getAbsolutePosition() - armEncoder.getPositionOffset()) * 360;
    }

    public void setAngle(double angle) {
        leftArm.setControl(request.withOutput(angleLoop.calculate(getAngle(), angle)/12));
        rightArm.setControl(request.withOutput(angleLoop.calculate(getAngle(), angle)/12));
    }
}
