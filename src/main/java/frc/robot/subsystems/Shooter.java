package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.PID;

public class Shooter {
    
    TalonFX shooterTop = new TalonFX(Constants.DeviceID.shooterBot);
    TalonFX shooterBot = new TalonFX(Constants.DeviceID.shooterTop);

    final VoltageOut request = new VoltageOut(0);
    double currentPower;
    PID rpmLoop = new PID(0, 0, 0);

    public void setPower(double power){
        shooterTop.set(power);
        shooterBot.set(power);
    }

    public double[] getRPM(){
        return new double[]{((600 * shooterTop.getRotorVelocity().getValue() / Constants.Motors.TalonFXCPR) * (24.0/18.0)),
                            ((600 * shooterBot.getRotorVelocity().getValue() / Constants.Motors.TalonFXCPR) * (24.0/18.0))};
    }

    public void setRPM(double rpm){
        rpmLoop.setSetpoint(rpm);
        rpmLoop.calculate(getRPM()[0]);
        rpmLoop.calculate(getRPM()[1]);
        shooterTop.setControl(request.withOutput(rpmLoop.getOutput()/12)); 
        shooterTop.setControl(request.withOutput(rpmLoop.getOutput()/12));
    }

    public void stopShooter() {
        rpmLoop.resetI();
        shooterTop.set(0); 
        shooterBot.set(0);
    }

    public double[] getTopVoltage() {    
        return new double[]{shooterTop.getSupplyVoltage().getValueAsDouble(),
                            shooterBot.getSupplyVoltage().getValueAsDouble()};
    }

}