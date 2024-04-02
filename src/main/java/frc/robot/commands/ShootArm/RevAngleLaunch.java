package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Shooter;

//ONLY USED FOR AUTOS
//USE RevAndAngle FOR TELEOP
public class RevAngleLaunch extends Command{
    
    private Arm arm;
    private Shooter shooter;
    private Intake intake;
    private double desiredAngle;
    private double stable;
    private Timer timer;
    private double rpm;

    public RevAngleLaunch(Arm arm, Shooter shooter, Intake intake, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
        desiredAngle = angle;
        timer = new Timer();
        this.rpm = 0;
    }

    public RevAngleLaunch(Arm arm, Shooter shooter, Intake intake, double angle, double rpm){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
        desiredAngle = angle;
        timer = new Timer();
        this.rpm = rpm;
    }

    @Override
    //Sets stable to 0 and starts the timer
    public void initialize(){
        stable = 0;
        timer.reset();
        timer.start();
    }

    @Override
    //Sets the arm to the desired angle. If the desired angle is the amp position, sets the shooter to the amp rpm, else sets the shooter to the regular shoot rpm.
    //If the arm is ready to shoot, increase stable by 1. If stable is greater than 5 or 2 seconds has gone by, run the intake to fire the note.
    public void execute(){

        if(arm.isReady()){
            stable++;
            System.out.println(stable);
        }
        if (desiredAngle == Positions.amp) shooter.ampRPM();
        else if (rpm != 0) shooter.setRPM(rpm);
        else shooter.shootRPM();
        arm.setShootPos(desiredAngle);
        if(stable > 5 || timer.hasElapsed(2.6)){
            intake.inTake();
        }
    }

    @Override
    //If the arm is in the amp position, give the note an extra few milliseconds to get out of the shooter. Then stop the intake and shooter.
    public void end(boolean interupt){
        if (desiredAngle == Positions.amp) Timer.delay(0.07);
        intake.stopTake();
        shooter.setRPM(0);
    }
    
    @Override
    //Finishes when there is no note in the intake
	public boolean isFinished() {
        return !NoteManager.hasNote();
	}
}
