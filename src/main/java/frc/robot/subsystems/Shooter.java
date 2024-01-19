package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.*;

import frc.robot.Constants;

public class Shooter {

    TalonFX shooter = new TalonFX(Constants.Ports.shooter);

    double currentPower;

    public void shoot(double power){
        shooter.set(power);
    }
}
