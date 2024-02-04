package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class MusicCommand extends Command{

    private SwerveSubsystem drivetrain;
    private Orchestra orchestra;
    private String songPath;

    public MusicCommand(SwerveSubsystem drivetrain, String songPath){
        this.drivetrain = drivetrain;
        this.songPath = songPath;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){

        orchestra.addInstrument(new TalonFX(3));
        orchestra.addInstrument(new TalonFX(5));
        orchestra.addInstrument(new TalonFX(7));
        orchestra.addInstrument(new TalonFX(9));
        orchestra.loadMusic(songPath);
        

    }

    @Override
    public void execute(){
        orchestra.play();
    }

    @Override
    public void end(boolean interrupted) {
        //drivetrain.stopSong();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
