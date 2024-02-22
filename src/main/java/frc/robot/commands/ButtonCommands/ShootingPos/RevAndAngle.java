package frc.robot.commands.ButtonCommands.ShootingPos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class RevAndAngle extends Command {

    private Arm arm;
    private Shooter shooter;
    private double desiredAngle;

    public RevAndAngle(Arm arm, Shooter shooter, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        desiredAngle = angle;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setRPM(Constants.Vision.shootRPM);
        arm.setPosition(desiredAngle);
    }

    @Override
    public void end(boolean interupt){
        arm.activeStop();
    }
    
    @Override
	public boolean isFinished() {
		return arm.isReady();
	}
}
