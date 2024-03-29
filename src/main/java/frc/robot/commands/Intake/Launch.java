package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
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
    //Runs the intake
    public void execute() { 
        intake.inTake();
    }

    @Override
    //Stops the intake after a few milliseconds
    public void end(boolean interupt) {
        Timer.delay(0.03);
        intake.stopTake();
	}

    @Override
    //Finishes when there is no note in the intake
	public boolean isFinished() {
		return !NoteManager.hasNote();
	}
}
