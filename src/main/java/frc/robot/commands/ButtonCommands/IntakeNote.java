package frc.robot.commands.ButtonCommands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;

public class IntakeNote extends Command {
    private Intake intake;

    private boolean started;

    public IntakeNote(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        started = false;
    }

    @Override
    public void execute() { 
        intake.inTake();
        if(intake.getRPM() > 1500){
            started = true;
        }
        if(intake.getRPM() < 1000 && started){
            //NoteManager.setTrue();
        }
    }

    @Override
    public void end(boolean interupt) {
        intake.outTake();
        Timer.delay(0.01);
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return false;//NoteManager.hasNote()
	}
}
