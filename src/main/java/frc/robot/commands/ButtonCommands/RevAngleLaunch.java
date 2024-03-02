package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RevAngleLaunch extends Command{
    
    private Arm arm;
    private Shooter shooter;
    private Intake intake;
    private double desiredAngle;
    private boolean ampMode;
    private double stable;

    public RevAngleLaunch(Arm arm, Shooter shooter, Intake intake, double angle){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
        desiredAngle = angle;
        ampMode = false;
    }

    public RevAngleLaunch(Arm arm, Shooter shooter, Intake intake, double angle, boolean ampMode){
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
        desiredAngle = angle;
        ampMode = true;
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
        double power = Constants.Vision.shootRPM;
        if(ampMode) power = 500;

        shooter.setRPM(power);
        arm.setPosition(desiredAngle);

        //System.out.println(stable);
    }

    @Override
    public void end(boolean interupt){
        intake.inTake();
        Timer.delay(0.3);
        intake.stopTake();
        shooter.setRPM(0);
    }
    
    @Override
	public boolean isFinished() {
        return arm.autoReady();
	}
}
