package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

public class Amp extends Command {

    private Arm arm;
    private Shooter shooter;
    private XboxController rumble;

    public Amp(Arm arm, Shooter shooter, XboxController rumble){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        this.rumble = rumble;
    }

    @Override
    public void initialize(){
    }

    @Override
    //If the intake has a note, rev up the shooter and set the arm to the amp position
    public void execute(){
        if(NoteManager.hasNote()){
            shooter.ampRPM();
            arm.setShootPos(Positions.amp);
        }
        if(arm.getPosition() > 0.48){
            rumble.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
        }
    }

    @Override
    //Wait a little and then stop the shooter so the note has time to get out
    public void end(boolean interupt){
        Timer.delay(0.3);
        shooter.setRPM(0);
        
    }
    
    @Override
    //Finishes when there is no note in the intake
	public boolean isFinished() {
        return !NoteManager.hasNote();
	}
}
