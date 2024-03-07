package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

    private Shooter shooter;

    public Shoot(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setRPM(Constants.Vision.shootRPM);
    }

    @Override
    public void end(boolean interupt){
        shooter.setRPM(0);
    }
    
    @Override
	public boolean isFinished() {
		return false;
	}
}
