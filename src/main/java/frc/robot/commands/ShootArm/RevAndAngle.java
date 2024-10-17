package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

public class RevAndAngle extends Command {

    private Arm arm;
    private Shooter shooter;
    private double desiredAngle;
    //private XboxController rumble;

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
    //If there is a note in the intake, set the shooter to 4500 rpm and set the arm the the desired position
    public void execute(){
        if(NoteManager.hasNote()){
            shooter.setRPM(5500);
            arm.setShootPos(desiredAngle);
            
        }
        if(arm.isReady()){
            //rumble.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
        }
    }

    @Override
    //Stops the shooter and gives a little delay for it to stop
    public void end(boolean interupt){
        shooter.setRPM(0);
        Timer.delay(0.01);
        
    }
    
    @Override
    //Finishes when there is no note in the intake
	public boolean isFinished() {
        return false;
	}
}
