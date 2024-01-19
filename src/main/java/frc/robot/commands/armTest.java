package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armTest extends Command{

    private Arm arm;

    public armTest(){
        arm = new Arm(Constants.Ports.armID);
    }

    @Override
    public void execute(){
        arm.runIntake();
        
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
    }

    @Override
    public boolean isFinished(){
       return false;
    }
}
