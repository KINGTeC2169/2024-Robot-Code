package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    CANSparkMax intakeMotor;

    public Intake(){
        intakeMotor = new CANSparkMax(Constants.DeviceID.intake, MotorType.kBrushless);
    }
    /**Sets intake to suck in */
    public void inTake() {
        intakeMotor.set(0.3);
        SmartDashboard.putBoolean("Intake", true);
    }

    /**Sets intake to outtake */
    public void outTake(boolean on) {
        intakeMotor.set(-0.3);
        SmartDashboard.putBoolean("Intake", false);
    }

    public void stopTake(){
        intakeMotor.set(0);
    }

}