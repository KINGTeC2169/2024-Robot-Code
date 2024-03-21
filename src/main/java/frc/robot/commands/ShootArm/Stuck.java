package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class Stuck extends Command {

    private Arm arm;
    private Shooter shooter;

    public Stuck(Arm arm, Shooter shooter){
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
        arm.setAmp();
    }

    @Override
    public void end(boolean interupt){
        shooter.ampRPM();
        Timer.delay(0.8);
        shooter.stopShooter();
    }
    
    @Override
	public boolean isFinished() {
        return arm.isReady();
	}
}
