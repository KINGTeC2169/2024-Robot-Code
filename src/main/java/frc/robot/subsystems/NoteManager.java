package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Ports;

public class NoteManager{
    
    private static DigitalInput beamBreak = new DigitalInput(Ports.beamBreak);

    public static boolean hasNote(){
        return !beamBreak.get();
    }
}
