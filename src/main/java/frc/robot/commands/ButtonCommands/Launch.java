package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;

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
        NoteManager.setFalse();
        intake.inTake();
    }

    @Override
    public void end(boolean interupt) {
        NoteManager.setFalse();
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return true;
	}
}
