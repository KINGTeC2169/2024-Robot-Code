package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class Safe extends Command {

    private Arm arm;
    private Intake intake;

    public Safe(Arm arm, Intake intake){
        this.arm = arm;
        addRequirements(arm);
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        arm.setAngle(10);
        intake.stopTake();
    }

    @Override
    public void end(boolean interupt){
        
    }
    
    @Override
	public boolean isFinished() {
		return true;
	}
}

