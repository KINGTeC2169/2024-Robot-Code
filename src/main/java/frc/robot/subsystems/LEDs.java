package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LEDs {   
    private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino);
//rainbow effect:
    public static void intialize(){
        arduino.writeString("A");
    }
//green:
    public static void green(){
        arduino.writeString("B");
    }
//red:
    public static void red(){
        arduino.writeString("C");
    }
//blue:
    public static void blue(){
        arduino.writeString("D");
    }
    public static void orange(){
        arduino.writeString("E");
    }
//turns leds off:
    public static void off(){
        arduino.writeString("F");
    }
}
