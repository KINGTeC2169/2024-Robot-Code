package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

    private Shooter shooter;
    private Intake intake;

    public Shoot(Shooter shooter, Intake intake){
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
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
        intake.inTake();
        Timer.delay(0.05);
        intake.stopTake();
        shooter.stopShooter();
    }
    
    @Override
	public boolean isFinished() {
		return false;
	}
}
