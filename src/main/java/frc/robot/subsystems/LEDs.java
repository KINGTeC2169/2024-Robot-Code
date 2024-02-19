package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LEDs {   
    private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino);

    public static void intialize(){
        arduino.writeString("A");
    }

    public static void red(){
        arduino.writeString("B");
    }

    public static void off(){
        arduino.writeString("Z");
    }
}
