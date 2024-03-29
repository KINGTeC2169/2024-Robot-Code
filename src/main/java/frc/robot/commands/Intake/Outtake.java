package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Outtake extends Command {
    private Intake intake;

    public Outtake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    //Runs intake backwards
    public void execute() { 
        intake.outTake();
    }

    @Override
    //Stops intake
    public void end(boolean interupt) {
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return false;
	}
}
