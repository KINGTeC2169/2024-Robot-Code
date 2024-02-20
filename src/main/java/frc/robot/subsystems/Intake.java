package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private TalonFX intakeMotor;
    private DigitalInput beamBreak;
    private GenericEntry intakeSpeed;
    private GenericEntry outtakeSpeed;

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    public Intake(){
        intakeMotor = new TalonFX(Constants.Ports.intake);
        beamBreak = new DigitalInput(Constants.Ports.beamBreak);

        tab.addDouble("Intake Voltage", () -> getVoltage()).withWidget(BuiltInWidgets.kVoltageView).withSize(2, 1).withPosition(0, 0).withProperties(Map.of("Max", 12));
        tab.addBoolean("Has note", () -> hasNote()).withPosition(2, 0);
        tab.addBoolean("Intake On", () -> isOn()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0);
        intakeSpeed = tab.add("Intake Speed", 0.2).withWidget(BuiltInWidgets.kNumberSlider).withPosition(4, 0).withSize(2, 1).withProperties(Map.of("Min", 0)).getEntry();
        outtakeSpeed = tab.add("Outtake Speed", 0.05).withWidget(BuiltInWidgets.kNumberSlider).withPosition(6, 0).withSize(2, 1).withProperties(Map.of("Min", 0)).getEntry();
        
    }
    /**Sets intake to suck in */
    public void inTake() {
        intakeMotor.set(-intakeSpeed.getDouble(0.2));
    }

    /**Sets intake to outtake */
    public void outTake() {
        intakeMotor.set(outtakeSpeed.getDouble(0.05));
    }

    public void stopTake(){
        intakeMotor.set(0);
    }

    public double getSpeed(){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public double getVoltage(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble();
    }

    public boolean isOn(){
        return Math.abs(getSpeed()) > 0;
    }

    public boolean hasNote(){
        return !beamBreak.get();
    }

}