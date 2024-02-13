package frc.robot.commands.ButtonCommands.ShootingPos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class Subwoofer extends Command {

    private Arm arm;
    private Shooter shooter;
    final private double desiredAngle = 45;

    public Subwoofer(Arm arm, Shooter shooter){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setRPM(Constants.Vision.shootRPM);
        arm.setAngle(desiredAngle);
    }

    @Override
    public void end(boolean interupt){
        arm.armStop();
    }
    
    @Override
	public boolean isFinished() {
		return arm.armReady() && shooter.shooterReady();
	}
}