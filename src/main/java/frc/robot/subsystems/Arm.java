package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Arm {
    
    private TalonFX motor;

    public Arm(int motorID){
        motor = new TalonFX(motorID);
    }

    public void run(){
        motor.set(-1);
    }

    public void runReverse(){
        motor.set(1);
    }

    public void stop(){
        motor.set(0);
    }
}
