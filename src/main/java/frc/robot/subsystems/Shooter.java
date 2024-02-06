package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    TalonFX shooterTop = new TalonFX(Constants.DeviceID.shooterBot);
    TalonFX shooterBot = new TalonFX(Constants.DeviceID.shooterTop);

    final VoltageOut request = new VoltageOut(0);
    double currentPower;
    PIDController rpmLoop = new PIDController(0, 0, 0);

    public void setPower(double power){
        shooterTop.set(power);
        shooterBot.set(power);
        SmartDashboard.putNumber("Shooting Power", power);
    }

    public double[] getRPM(){
        return new double[]{((600 * shooterTop.getRotorVelocity().getValue() / Constants.Motors.TalonFXCPR) * (24.0/18.0)),
                            ((600 * shooterBot.getRotorVelocity().getValue() / Constants.Motors.TalonFXCPR) * (24.0/18.0))};
    }

    public void setRPM(double rpm){
        shooterTop.setControl(request.withOutput(rpmLoop.calculate(getRPM()[0], rpm)/12)); 
        shooterTop.setControl(request.withOutput(rpmLoop.calculate(getRPM()[1], rpm)/12));
    }

    public void stopShooter() {
        rpmLoop.reset();
        shooterTop.set(0); 
        shooterBot.set(0);
    }

    public double[] getTopVoltage() {    
        return new double[]{shooterTop.getSupplyVoltage().getValueAsDouble(),
                            shooterBot.getSupplyVoltage().getValueAsDouble()};
    }

}