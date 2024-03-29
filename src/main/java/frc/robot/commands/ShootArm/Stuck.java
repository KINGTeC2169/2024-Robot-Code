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
    //Sets the arm to the amp scoring position
    public void execute(){
        arm.setAmp();
    }

    @Override
    //Runs the shooter to get the note out of it and then stops the shooter
    public void end(boolean interupt){
        arm.setVoltage(0);
        shooter.ampRPM();
        Timer.delay(0.8);
        shooter.stopShooter();
    }
    
    @Override
    //TODO: better return statement
	public boolean isFinished() {
        return false;
	}
}
