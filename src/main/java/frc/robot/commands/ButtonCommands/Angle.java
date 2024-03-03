package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Angle extends Command {

    private Arm arm;
    private double angle;

    public Angle(Arm arm, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.angle = angle;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        arm.setPosition(angle);
    }

    @Override
    public void end(boolean interupt){
    }
    
    @Override
	public boolean isFinished() {
		return false;
	}
}
