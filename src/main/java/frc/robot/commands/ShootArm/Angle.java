package frc.robot.commands.ShootArm;

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
    //Set the arm to the supplied postion
    public void execute(){
        arm.setShootPos(angle);
    }

    @Override
    public void end(boolean interupt){
    }
    
    @Override
	public boolean isFinished() {
		return false;
	}
}
