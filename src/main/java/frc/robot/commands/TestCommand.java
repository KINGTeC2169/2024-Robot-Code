package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TestCommand extends Command{

    private SwerveSubsystem drivetrain;
    private String songPath;

    public TestCommand(SwerveSubsystem drivetrain, String songPath){
        this.drivetrain = drivetrain;
        this.songPath = songPath;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        //drivetrain.playSong(songPath);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted) {
        //drivetrain.stopSong();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
