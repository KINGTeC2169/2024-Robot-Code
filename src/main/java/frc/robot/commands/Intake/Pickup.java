package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;

public class Pickup extends Command {
    private Intake intake;

    public Pickup(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    //Runs intake
    public void execute() { 
        intake.inTake();
    }

    @Override
    //This is a small adjustment of the note in case the beambreak is too slow. It runs the intake backwards for a few milliseconds to make sure the note doesn't touch the shooter wheels.
    public void end(boolean interupt) {
        intake.stopTake();
        intake.outTake();
        Timer.delay(0.02);
        intake.stopTake();
	}

    @Override
    //Returns true if there is a note in the intake
	public boolean isFinished() {
		return NoteManager.hasNote();
	}
}
