package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armTest extends Command{

    private Arm arm;
    private Arm arm1;

    public armTest(){
        arm = new Arm(Constants.Ports.armID);
        arm1 = new Arm(Constants.Ports.armID1);
    }

    @Override
    public void execute(){
        arm.runIntake();
        arm1.runIntake();
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        arm1.stop();
    }

    @Override
    public boolean isFinished(){
       return false;
    }
}
