package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

public class Launch extends Command {
    private Intake intake;
    private boolean started;

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
	}

    @Override
	public boolean isFinished() {
		return false;//!NoteManager.hasNote();
	}
}
