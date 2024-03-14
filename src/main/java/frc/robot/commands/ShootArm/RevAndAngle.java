package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

public class RevAndAngle extends Command {

    private Arm arm;
    private Shooter shooter;
    private double desiredAngle;

    public RevAndAngle(Arm arm, Shooter shooter, double angle){
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
        if(NoteManager.hasNote()){
            shooter.setRPM(6000);
            arm.setShootPos(desiredAngle);
        }
    }

    @Override
    public void end(boolean interupt){
        shooter.setRPM(0);
        Timer.delay(0.01);
    }
    
    @Override
	public boolean isFinished() {
        return !NoteManager.hasNote();
	}
}
