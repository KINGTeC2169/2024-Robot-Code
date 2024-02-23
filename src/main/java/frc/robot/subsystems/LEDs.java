package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LEDs {   
    private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino);

    public static void intialize(){
        arduino.writeString("A");
    }

    public static void green(){
        arduino.writeString("B");
    }

    public static void red(){
        arduino.writeString("C");
    }

    public static void blue(){
        arduino.writeString("D");
    }

    public static void off(){
        arduino.writeString("E");
    }
}
