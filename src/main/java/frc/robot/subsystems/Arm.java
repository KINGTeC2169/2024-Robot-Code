package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm {
    
    private TalonFX motor;
    private RelativeEncoder motorEncoder;

    public Arm(int motorID){
        motor = new TalonFX(motorID);

        resetEncoder();
    }

    public void runIntake(){
        motor.set(0.75);
    }

    public void resetEncoder(){
        motorEncoder.setPosition(0);
    }

    public void stop(){
        motor.set(0);
    }
}
