package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmStop extends Command{
    private Arm arm;

    public ArmStop(Arm arm){
        this.arm = arm;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        arm.armStop();
    }

    @Override
    public void end(boolean interupt){
    }
    
    @Override
	public boolean isFinished() {
		return arm.off();
	}
}
