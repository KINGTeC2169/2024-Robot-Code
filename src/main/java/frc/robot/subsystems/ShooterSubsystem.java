package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class ShooterSubsystem {

    private Shooter shooterTop = new Shooter(Constants.DeviceID.shooterTop);
    private Shooter shooterBot = new Shooter(Constants.DeviceID.shooterBot);

    private ShuffleboardTab sTab = Shuffleboard.getTab("Shooter");

    public void stopShooter(){
        shooterTop.stopShooter();
        shooterBot.stopShooter();
    }

    public void setRPM(double rpm){
        shooterTop.setRPM(rpm);
        shooterBot.setRPM(-rpm);
    }
}
