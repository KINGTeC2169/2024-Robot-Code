package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Modes;

public class Intake extends SubsystemBase{
    
    private TalonFX intakeMotor;

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    public Intake(){
        intakeMotor = new TalonFX(Constants.Ports.intake);

        tab.addDouble("Intake Voltage", () -> getVoltage()).withWidget(BuiltInWidgets.kVoltageView).withSize(2, 1).withPosition(0, 0).withProperties(Map.of("Max", 12));
        tab.addBoolean("Has note", () -> NoteManager.hasNote()).withPosition(2, 0);
        tab.addBoolean("Intake On", () -> isOn()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0);
        
        tab.addDouble("Intake RPM", () -> getRPM()).withPosition(0, 1);
        tab.addBoolean("Note Manager Distance", () -> NoteManager.hasNote()).withPosition(0,2);
        tab.addDouble("Current Mode", () -> getMode()).withPosition(4,0);
    }

    public int getMode(){
        return Modes.intakeMode;
    }

    public void cycleMode(){
        Modes.intakeMode++;
        Modes.intakeMode%=3;
    }

    /**Sets intake to suck in */
    public void inTake() {
        intakeMotor.set(-0.25);
    }

    /**Returns the intake motor's rotor velocity in rotations per minute */
    public double getRPM(){
        return -(60 * intakeMotor.getRotorVelocity().getValue());
    }

    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        intakeMotor.set(0.12);
    }

    /**Stops the intake. */
    public void stopTake(){
        intakeMotor.set(0);
    }

    /**Returns the velocity of the intake motor. */
    public double getSpeed(){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    /**Returns the voltage of the intake motor. */
    public double getVoltage(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble();
    }

    /**Returns the current of the intake motor. */
    public double getCurrent(){
        return intakeMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**Returns true of the intake is on. */
    public boolean isOn(){
        return Math.abs(getSpeed()) > 0;
    }

    /**Used for music commands */
    public void playNote(double hz){
        intakeMotor.setControl(new MusicTone(hz));
    }
}