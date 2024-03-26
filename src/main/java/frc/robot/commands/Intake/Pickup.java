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
    public void execute() { 
        intake.inTake();
    }

    @Override
    public void end(boolean interupt) {
        intake.stopTake();
        intake.outTake();
        Timer.delay(0.02);
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return NoteManager.hasNote();
	}
}
