package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class NoteManager{
    
    private static boolean note = false;
    private static Rev2mDistanceSensor sensor = new Rev2mDistanceSensor(Port.kOnboard);
    
    public static void startUp(){
        sensor.setAutomaticMode(true);
    }

    public static double getDistance(){
        return sensor.getRange();
    }

    /**This tells the shooter that there isn't a note in the intake */
    public static void setFalse(){
        note = false;
    }

    /**This tells the shooter that there is a not in the intake */
    public static void setTrue(){
        note = true;
    }

    /**Returns whether or not a note is in the intake */
    public static boolean hasNote(){
        return note;
    }
}
