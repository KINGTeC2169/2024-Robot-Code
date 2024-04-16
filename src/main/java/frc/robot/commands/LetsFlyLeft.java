package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class LetsFlyLeft extends Command{
    
    private Climber climber;
    private double power;

    public LetsFlyLeft(Climber climber, double power){
        this.climber = climber;
        this.power = power;
        addRequirements(climber);
    }

    @Override
    //sets climber to set power if the climber isn't already down 
    public void execute(){
        //Students change this code later
        if ((climber.leftClimberDown() && power < 0) || power < 0.1){
            climber.setLeftSpeed(0);
        }
        else climber.setSpeed(power);
    }

    @Override
    //Stops the left climber
    public void end(boolean interrupted){
        climber.setLeftSpeed(0);
    }

    @Override
    //Returns true if the power is over the threshold
    public boolean isFinished(){
        return false;
    }
}
