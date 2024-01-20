package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Claw {
    
    private CANSparkMax motor;
    private RelativeEncoder motorEncoder;

    public Claw(int motorID){
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        motorEncoder = motor.getEncoder();

        resetEncoder();
    }

    public void runIntake(){
        motor.set(0.25);
    }

    public void resetEncoder(){
        motorEncoder.setPosition(0);
    }

    public void stop(){
        motor.set(0);
    }
}
