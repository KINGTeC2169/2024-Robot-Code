package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.PID;

public class Shooter {
    
    TalonFX shooter;

    final VoltageOut request = new VoltageOut(0);
    double currentPower;
    PID rpmLoop = new PID(0, 0, 0);

    public Shooter(int id){
        shooter = new TalonFX(id);
    }

    public void setPower(double power){
        shooter.set(power);
    }

    public double getRPM(){
        return 600; //I'll work on this today dw
    }

    public void setRPM(double rpm){
        rpmLoop.setSetpoint(rpm);
        rpmLoop.calculate(getRPM());
        shooter.setControl(request.withOutput(rpmLoop.getOutput()/12));  
    }
    
    public void stopShooter() {
        rpmLoop.resetI();
        shooter.set(0); 
    }

    public double getTopCurrent() {
        return shooter.getSupplyCurrent().getValueAsDouble();
    }   
    public double getTopVoltage() {    
        return shooter.getSupplyVoltage().getValueAsDouble();
    }

}