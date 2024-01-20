package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class intakeTest extends Command{

    private Claw intake;

    public intakeTest(){
        intake = new Claw(Constants.DeviceID.intake);
    }

    @Override
    public void execute(){
        intake.runIntake();   
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    @Override
    public boolean isFinished(){
       return false;
    }
}
