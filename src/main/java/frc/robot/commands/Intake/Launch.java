package frc.robot.commands.Intake;

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
        intake.inTake();
        if(intake.getRPM() > 1500){
            NoteManager.setFalse();
        }
    }

    @Override
    public void end(boolean interupt) {
        intake.stopTake();
        NoteManager.setFalse();
	}

    @Override
	public boolean isFinished() {
		return false;
	}
}
