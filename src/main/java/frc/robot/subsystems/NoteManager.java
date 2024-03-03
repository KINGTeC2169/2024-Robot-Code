package frc.robot.subsystems;

public class NoteManager{
    
    private static boolean note = false;

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
