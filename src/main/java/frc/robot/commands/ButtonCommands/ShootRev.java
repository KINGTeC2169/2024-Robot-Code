package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootRev extends Command {

    private Shooter shooter;

    public ShootRev(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setRPM(2000);
    }

    @Override
    public void end(boolean interupt){
        shooter.stopShooter();
    }
    
    @Override
	public boolean isFinished() {
		return true;
	}
}
