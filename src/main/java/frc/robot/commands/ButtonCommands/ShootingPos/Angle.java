package frc.robot.commands.ButtonCommands.ShootingPos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class Angle extends Command {

    private Arm arm;
    private CommandXboxController controller;

    //It can be changed later to speed with a joystick on the right side
    public Angle(Arm arm, CommandXboxController controller){
        this.arm = arm;
        addRequirements(arm);
        this.controller = controller;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){

        /* 
        if(!(Math.abs(controller.getRightY()) < 0.05)){
        //speed = controller.getRightY();
        
            arm.setAngleNoPID(-controller.getRightY()/2.0);
        
        } else{
            arm.setAngleNoPID(0);
        }
        
        arm.setAngle(speed*25);
        //arm.setAngleNoPID(speed);
        */

        arm.setSpeed(controller.getRightY()*-1);
    }

    @Override
    public void end(boolean interupt){
        arm.activeStop();
    }
    
    @Override
	public boolean isFinished() {
		return arm.isReady();
	}
}
