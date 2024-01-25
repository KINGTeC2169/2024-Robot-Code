package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    CANSparkMax intakeMotor = new CANSparkMax(Constants.DeviceID.intake, MotorType.kBrushless);

    /**Sets intake to suck in */
    public void inTake(boolean on) {
        if(on) intakeMotor.set(0.3);
        else intakeMotor.set(0);
        SmartDashboard.putBoolean("Intake", true);
    }

    /**Sets intake to outtake */
    public void outTake(boolean on) {
        if(on) intakeMotor.set(-0.3);
        else intakeMotor.set(0);
        SmartDashboard.putBoolean("Intake", false);
    }

}