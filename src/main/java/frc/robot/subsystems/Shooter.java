package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    TalonFX shooterTop = new TalonFX(Constants.DeviceID.shooterBot);
    TalonFX shooterBot = new TalonFX(Constants.DeviceID.shooterTop);

    double setRPM;
    double setPower;

    final VoltageOut request = new VoltageOut(0);
    double currentPower;
    PIDController rpmLoop = new PIDController(0, 0, 0);

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    

    public Shooter(){

        var configs = new Slot0Configs();
        configs.kP = 0.1;
        
        shooterTop.getConfigurator().apply(configs,0.050);
        shooterBot.getConfigurator().apply(configs,0.050);

        setRPM = 0;

        tab.addBoolean("Shooter Ready", () -> shooterReady());
        tab.addDouble("Shoot RPM Top", () -> getRPM()[0]);
        tab.addDouble("Shoot RPM Bot", () -> getRPM()[1]);
        tab.addDouble("Shoot Power Top", () -> getShootCurrent()[0]);
        tab.addDouble("Shoot Power Bot", () -> getShootCurrent()[1]);
        

    }

    public void setPower(double power){
        shooterTop.set(power);
        shooterBot.set(power);
        SmartDashboard.putNumber("Request ShotPow", power);
    }

    public double[] getShootCurrent(){
        return new double[]{shooterTop.getSupplyCurrent().getValueAsDouble(),
                            shooterBot.getSupplyCurrent().getValueAsDouble()}; 
    }

    public double[] getRPM(){
        return new double[]{((600 * shooterTop.getRotorVelocity().getValue() / Constants.Motors.TalonFXCPR) * (24.0/18.0)),
                            ((600 * shooterBot.getRotorVelocity().getValue() / Constants.Motors.TalonFXCPR) * (24.0/18.0))};
    }

    public void setRPM(double rpm){

        setRPM = rpm;

        shooterTop.setControl(request.withOutput(rpmLoop.calculate(getRPM()[0], rpm)/12)); 
        shooterTop.setControl(request.withOutput(rpmLoop.calculate(getRPM()[1], rpm)/12));
    }

    public boolean shooterReady(){
        return Math.abs(getRPM()[0]-setRPM) < 30 && Math.abs(getRPM()[1]-setRPM) < 30;
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