package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;

public class Shooter extends SubsystemBase {
    
    private TalonFX shooterTop;
    private TalonFX shooterBot;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private GenericEntry shooterSpeed = tab.add("Shooter Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(4, 0).getEntry();

    public Shooter(){
        
        shooterTop = new TalonFX(Constants.Ports.shooterTop);
        shooterBot = new TalonFX(Constants.Ports.shooterBot);

        var configs = new Slot0Configs();
        configs.kP = 0.1;

        shooterTop.getConfigurator().apply(configs,0.050);
        shooterBot.getConfigurator().apply(configs,0.050);

        ShuffleboardLayout topMotor = tab.getLayout("Top Motor", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
        topMotor.addDouble("Top Motor RPM", () -> getRPM()[0]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 4000));
        topMotor.addDouble("Top Motor Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max", 12));
        topMotor.addDouble("Top Motor Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial);

        ShuffleboardLayout bottomMotor = tab.getLayout("Bottom Motor", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
        bottomMotor.addDouble("Bottom Motor RPM", () -> getRPM()[1]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 4000));
        bottomMotor.addDouble("Bottom Motor Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max", 12));
        bottomMotor.addDouble("Bottom Motor Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial);

        tab.addBoolean("Shooter Ready", () -> shooterReady()).withPosition(6, 0);

    }

    /**Sets the power as a double between -1 and 1*/
    
    public void setPower(double power){
        shooterTop.set(-power);
        shooterBot.set(-power);
        SmartDashboard.putNumber("Request ShotSpeed", power);
    }

    /**Gets the current from the top and bottom motors as a double array */
    public double[] getCurrent(){
        return new double[]{shooterTop.getSupplyCurrent().getValueAsDouble(),
                            shooterBot.getSupplyCurrent().getValueAsDouble()};
    }

    /**Gets the voltage from the top and bottom motors as a double array */
    public double[] getVoltage() {    
        return new double[]{shooterTop.getSupplyVoltage().getValueAsDouble(),
                            shooterBot.getSupplyVoltage().getValueAsDouble()};
    }
    
    /**Gets the rpm from the top and bottom motors as a double array*/
    public double[] getRPM(){
        return new double[]{((60 * shooterTop.getRotorVelocity().getValue())),
                            ((60 * shooterBot.getRotorVelocity().getValue()))};
    }

    public void setRPM(double rpm){

        double power = rpm/6350;
        setPower(power);
    }

    public void setRPM(){
        
        setPower(shooterSpeed.getDouble(0.5));
    }

    public void setAmpRPM(){
        setPower(0.1);
    }


    public boolean shooterReady(){
        return getRPM()[0] > Vision.shootRPM && getRPM()[1] > Vision.shootRPM;
    }

    public void stopShooter() {
        shooterTop.set(0); 
        shooterBot.set(0);
    }

    public boolean on(){
        return getCurrent()[0] > 0 || getCurrent()[1] > 0;
    }
}