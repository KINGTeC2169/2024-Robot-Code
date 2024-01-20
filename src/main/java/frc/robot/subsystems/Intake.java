package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Intake {
    
    CANSparkMax intakeMotor = new CANSparkMax(Constants.DeviceID.intake, MotorType.kBrushless);

    /**Sets intake to suck in */
    public void inTake(boolean on) {
        if(on) intakeMotor.set(0.3);
        else intakeMotor.set(0);
    }

    /**Sets intake to outtake */
    public void outTake(boolean on) {
        if(on) intakeMotor.set(-0.3);
        else intakeMotor.set(0);
    }

}