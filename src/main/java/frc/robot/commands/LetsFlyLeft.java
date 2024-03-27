package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class LetsFlyLeft extends Command{
    
    private Climber climber;

    public LetsFlyLeft(Climber climber){
        this.climber = climber;
    }

    @Override
    public void execute(){
        climber.setLeftMaxHeight();
        climber.setLeftSpeed(-0.2);
    }

    @Override
    public void end(boolean interrupted){
        climber.setLeftSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return climber.leftClimberDown();
    }
}
