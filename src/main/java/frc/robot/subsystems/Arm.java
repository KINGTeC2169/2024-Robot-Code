package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm {
    
    private TalonFX motor;

    public Arm(int motorID){
        motor = new TalonFX(motorID);
    }

    public void runIntake(){
        motor.set(1);
    }

    public void stop(){
        motor.set(0);
    }
}
