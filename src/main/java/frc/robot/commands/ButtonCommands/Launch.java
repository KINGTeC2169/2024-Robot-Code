package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Launch extends Command {
    private Intake intake;

    public Launch(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() { 
        intake.inTake(true);
    }

    @Override
    public void end(boolean interupt) {
		intake.inTake(false);
	}

    @Override
	public boolean isFinished() {
		return true;
	}
}
