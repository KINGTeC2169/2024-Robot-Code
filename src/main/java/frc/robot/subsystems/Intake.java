package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private TalonFX intakeMotor;

    public Intake(){
        intakeMotor = new TalonFX(Constants.DeviceID.intake);
    }
    /**Sets intake to suck in */
    public void inTake() {
        intakeMotor.set(0.3);
        SmartDashboard.putBoolean("Intake", true);
    }

    /**Sets intake to outtake */
    public void outTake() {
        intakeMotor.set(-0.3);
        SmartDashboard.putBoolean("Intake", false);
    }

    public void stopTake(){
        intakeMotor.set(0);
    }

    public double getSpeed(){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public boolean off(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble() == 0;
    }

    public boolean on(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble() != 0;
    }

}