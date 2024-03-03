package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmStop extends Command{
    private Arm arm;
    private double position;

    public ArmStop(Arm arm){
        this.arm = arm;
    }

    @Override
    public void initialize(){
        position = arm.getPosition();
    }

    @Override
    public void execute(){
        arm.setPosition(position);
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
