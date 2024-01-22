package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Angle extends Command {

    private Arm arm;
    private double up;

    //It can be changed later to speed with a joystick on the right side
    public Angle(Arm arm, boolean up){
        this.arm = arm;
        addRequirements(arm);
        this.up = (up ? 1 : -1);
    }

    public Angle(Arm arm, double speed){
        
    }


    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        arm.setAngle(arm.getAngle() + up);
    }

    @Override
    public void end(boolean interupt){
        arm.armStop();
    }
    
    @Override
	public boolean isFinished() {
		return true;
	}
}
