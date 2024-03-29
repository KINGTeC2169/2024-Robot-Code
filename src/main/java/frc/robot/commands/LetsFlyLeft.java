package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class LetsFlyLeft extends Command{
    
    private Climber climber;

    public LetsFlyLeft(Climber climber){
        this.climber = climber;
    }

    @Override
    //Raises left climber to max height and starts lowering it
    public void execute(){
        climber.setLeftMaxHeight();
        climber.setLeftSpeed(-0.2);
    }

    @Override
    //Stops the left climber
    public void end(boolean interrupted){
        climber.setLeftSpeed(0);
    }

    @Override
    //Returns true if the touch sensor on the left climber is pressed
    public boolean isFinished(){
        return climber.leftClimberDown();
    }
}
