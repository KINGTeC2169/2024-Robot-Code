package frc.robot.subsystems;

public class NoteManager{
    
    private static boolean note;

    public static void setFalse(){
        note = false;
    }

    public static void setTrue(){
        note = true;
    }

    public static boolean hasNote(){
        return note;
    }
}
