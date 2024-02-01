package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class Angle extends Command {

    private Arm arm;
    private double speed;
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
        speed = controller.getLeftY();
        arm.setAngleNoPID(speed);
    }

    @Override
    public void end(boolean interupt){
        arm.armStop();
    }
    
    @Override
	public boolean isFinished() {
		return true;
	}
}
