package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    TalonFX leftArm = new TalonFX(Constants.DeviceID.leftArm);
    TalonFX rightArm = new TalonFX(Constants.DeviceID.rightArm);

    DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.DeviceID.armEncoder);


    final double armLowerLimit = 0;
    final double armUpperLimit = 45;

    public double getAngle() {
        return (armEncoder.getAbsolutePosition() - armEncoder.getPositionOffset()) * 360;
    }

    public void setAngle(double angle) {
        double curAngle = getAngle();
        if(angle-curAngle > 1){
            leftArm.set(1);
            rightArm.set(-1);
        } else if(angle-curAngle < -1){
            leftArm.set(-1);
            rightArm.set(1);
        }
    }
}
