package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

public class RevAngleLaunch extends Command{
    
    private Arm arm;
    private Shooter shooter;
    private Intake intake;
    private double desiredAngle;
    private double stable;

    public RevAngleLaunch(Arm arm, Shooter shooter, Intake intake, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
        desiredAngle = angle;
    }

    @Override
    public void initialize(){
        stable = 0;
    }

    @Override
    public void execute(){

        if(arm.isReady()){
            stable++;
            System.out.println(stable);
        }

        shooter.shootRPM();
        arm.setShootPos(desiredAngle);

        if(stable>3 && shooter.shooterReady()){
            intake.inTake();
        }
    }

    @Override
    public void end(boolean interupt){
        intake.stopTake();
        shooter.setRPM(0);
    }
    
    @Override
	public boolean isFinished() {
        return !NoteManager.hasNote();
	}
}
