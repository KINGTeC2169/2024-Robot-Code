package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class Podium extends Command {

    private Arm arm;
    private Shooter shooter;
    final private double desiredAngle = 20;

    public Podium(Arm arm, Shooter shooter){
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
        shooter.setRPM(2000);
        arm.setAngle(desiredAngle);
    }

    @Override
    public void end(boolean interupt){
        shooter.stopShooter();
    }
    
    @Override
	public boolean isFinished() {
		return true;
	}
}
