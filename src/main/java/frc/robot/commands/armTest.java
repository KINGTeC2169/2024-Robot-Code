package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armTest extends Command{

    private Arm bottom;
    private Arm top;

    public armTest(){
        bottom = new Arm(Constants.DeviceID.shooterBot);
        top = new Arm(Constants.DeviceID.shooterTop);
    }

    @Override
    public void execute(){
        bottom.run();
        top.runReverse();
    }

    @Override
    public void end(boolean interrupted){
        bottom.stop();
        top.stop();
    }

    @Override
    public boolean isFinished(){
       return false;
    }
}
