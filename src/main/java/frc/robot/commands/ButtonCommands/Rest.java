package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.Arm;

public class Rest extends Command {

    private Arm arm;

    public Rest(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        Timer.delay(0.05);
    }

    @Override
    public void execute(){
        arm.setRest();
    }

    @Override
    public void end(boolean interupt){
        arm.setSpeed(0);
    }
    
    @Override
	public boolean isFinished() {
		return arm.isReady();
	}
}


