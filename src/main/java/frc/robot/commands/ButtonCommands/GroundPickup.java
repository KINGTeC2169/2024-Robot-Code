package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class GroundPickup extends Command {
    private Arm arm;
    private Intake intake;

    public GroundPickup(Arm arm, Intake intake){
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
        new Rest(arm);
        new Pickup(intake);
    }

    @Override
    public void end(boolean interupt){
        new Rest(arm);
    }
    
    @Override
	public boolean isFinished() {
		return false;
	}
}
