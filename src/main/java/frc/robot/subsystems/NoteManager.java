package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Ports;

public class NoteManager{
    
    private static DigitalInput beamBreak = new DigitalInput(Ports.beamBreak);
    private static boolean hasNote = false;
    private static int intakeMode = 2;

    /**Returns true if the intake has a note. */
    public static boolean hasNote(){
        //if(intakeMode == 2) 
        return !beamBreak.get();
        //return hasNote;
    }

    public static void setTrue(){
        hasNote = true;
    }

    public static void setFalse(){
        hasNote = false;
    }
}
