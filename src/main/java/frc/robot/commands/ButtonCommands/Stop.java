package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Stop extends Command{
    private Shooter shooter;
    private Intake intake;

    public Stop(Arm arm, Shooter shooter, Intake intake){
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
        shooter.stopShooter();
        intake.stopTake();
    }

    @Override
    public void end(boolean interupt){
    }
    
    @Override
	public boolean isFinished() {
		return shooter.off() && intake.off();
	}
}
