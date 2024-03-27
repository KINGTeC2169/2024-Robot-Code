package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class LetsFlyRight extends Command{
    
    private Climber climber;

    public LetsFlyRight(Climber climber){
        this.climber = climber;
    }

    @Override
    public void execute(){
        climber.setRightMaxHeight();
        climber.setRightSpeed(-0.2);
    }

    @Override
    public void end(boolean interrupted){
        climber.setRightSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return climber.rightClimberDown();
    }
}
