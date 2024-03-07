package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

public class Amp extends Command {

    private Arm arm;
    private Shooter shooter;

    public Amp(Arm arm, Shooter shooter){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        NoteManager.setTrue();
    }

    @Override
    public void execute(){
        shooter.ampRPM();
        arm.setAmp();
    }

    @Override
    public void end(boolean interupt){
        shooter.setRPM(0);
    }
    
    @Override
	public boolean isFinished() {
        return !NoteManager.hasNote();
	}
}
