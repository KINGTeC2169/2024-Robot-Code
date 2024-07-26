package frc.robot.commands.ShootArm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.Arm;

public class Rest extends Command {

    private Arm arm;
    //private XboxController rumble;

    public Rest(Arm arm){
        this.arm = arm;
        addRequirements(arm);
        //this.rumble = rumble;
    }

    @Override
    //Adds extra delay so the previous command can finish
    public void initialize(){
        Timer.delay(0.05);
        //rumble.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    @Override
    //If the arm is at the amp position, go to rest faster. Otherwise, slowly move the arm to the rest position
    public void execute(){
        arm.setRest(true);
        //if (arm.getSetPosition() == Positions.amp) arm.setRest(true);
        //else arm.setRest(false);
    }

    @Override
    //Set the arm voltage to 0 to stop its movement
    public void end(boolean interupt){
        arm.setVoltage(0);
    }
    
    @Override
    //If the arm is in rest position, return true
	public boolean isFinished() {
		return arm.restReady();
	}
}


