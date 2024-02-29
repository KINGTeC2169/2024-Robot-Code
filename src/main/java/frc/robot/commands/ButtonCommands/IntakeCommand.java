package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private Intake intake;
    private Arm arm;

    public IntakeCommand(Intake intake, Arm arm){
        this.intake = intake;
        addRequirements(this.intake);
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        System.out.println(arm.getPosition());
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
