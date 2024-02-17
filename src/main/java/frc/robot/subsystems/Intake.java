package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private TalonFX intakeMotor;

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    public Intake(){
        intakeMotor = new TalonFX(Constants.DeviceID.intake);

        tab.addDouble("Intake Volt", () -> getVoltage()).withWidget(BuiltInWidgets.kVoltageView);
        tab.addBoolean("Intake On", () -> on()).withWidget(BuiltInWidgets.kVoltageView);

    }
    /**Sets intake to suck in */
    public void inTake() {
        intakeMotor.set(-0.18);
        SmartDashboard.putBoolean("Intake", true);
    }

    /**Sets intake to outtake */
    public void outTake() {
        intakeMotor.set(0.18);
        SmartDashboard.putBoolean("Intake", false);
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

    public boolean off(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble() <= 5; //Change these values to resting voltage
    }

    public boolean on(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble() > 5; //Change these values to active voltage
    }

}