package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class LetsFlyRight extends Command{
    
    private Climber climber;

    public LetsFlyRight(Climber climber){
        this.climber = climber;
    }

    @Override
    //Raises right climber to max height and starts lowering it
    public void execute(){
        climber.setRightMaxHeight();
        climber.setRightSpeed(-0.2);
    }

    @Override
    //Stops the right climber
    public void end(boolean interrupted){
        climber.setRightSpeed(0);
    }

    @Override
    //Returns true if the touch sensor on the right climber is pressed
    public boolean isFinished(){
        return climber.rightClimberDown();
    }
}
