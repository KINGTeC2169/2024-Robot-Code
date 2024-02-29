package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;

public class Pickup extends Command {
    private Intake intake;

    private boolean started;

    public Pickup(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        started = false;
        NoteManager.setFalse();
    }

    @Override
    public void execute() { 
        intake.inTake();
        if(intake.getRPM() > 1500){
            started = true;
        }
        if(intake.getRPM() < 800 && started){
            NoteManager.setTrue();
        }
    }

    @Override
    public void end(boolean interupt) {
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return false || NoteManager.hasNote();
	}
}
