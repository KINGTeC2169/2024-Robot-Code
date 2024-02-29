package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class RevAndAim extends Command {

    private Arm arm;
    private Shooter shooter;
    private double desiredAngle;

    public RevAndAim(Arm arm, Shooter shooter, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        desiredAngle = angle;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setRPM(Constants.Vision.shootRPM);
        arm.setAim(desiredAngle);
    }

    @Override
    public void end(boolean interupt){
        new Angle(arm, Positions.rest);
        shooter.setRPM(0);
    }
    
    @Override
	public boolean isFinished() {
		return false;
	}
}
