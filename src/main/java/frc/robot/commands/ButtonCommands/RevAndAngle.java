package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class RevAndAngle extends Command {

    private Arm arm;
    private Shooter shooter;
    private double desiredAngle;
    private boolean ampMode;

    public RevAndAngle(Arm arm, Shooter shooter, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        desiredAngle = angle;
        ampMode = false;
    }

    public RevAndAngle(Arm arm, Shooter shooter, double angle, boolean ampMode){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        desiredAngle = angle;
        this.ampMode = true;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(ampMode) shooter.setAmpRPM();
        else shooter.setRPM(Vision.shootRPM);
        arm.setPosition(desiredAngle);
    }

    @Override
    public void end(boolean interupt){
        shooter.setRPM(0);
    }
    
    @Override
	public boolean isFinished() {
        return false;
	}
}
