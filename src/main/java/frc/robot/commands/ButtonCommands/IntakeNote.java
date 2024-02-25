package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    private Intake intake;

    public IntakeNote(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() { 
        intake.inTake();
    }

    @Override
    public void end(boolean interupt) {
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return true;
	}
}
