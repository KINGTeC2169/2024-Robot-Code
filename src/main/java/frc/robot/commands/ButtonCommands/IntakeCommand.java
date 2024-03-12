package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private Intake intake;

    public IntakeCommand(Intake intake){
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize(){
        //System.out.println(arm.getPosition());
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
		return false;
	}
}
