package frc.robot.commands.ButtonCommands.ShootingPos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class Amp extends Command {

    private Arm arm;
    private Shooter shooter;
    final private double desiredAngle = 90;

    public Amp(Arm arm, Shooter shooter){
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
        shooter.setAmpRPM(Constants.Vision.shootRPM);
        arm.setPosition(desiredAngle);
    }

    @Override
    public void end(boolean interupt){
        arm.activeStop();
    }
    
    @Override
	public boolean isFinished() {
		return arm.isReady() && shooter.shooterReady();
	}
}
