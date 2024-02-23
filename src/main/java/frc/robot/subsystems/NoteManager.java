package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class NoteManager{
    
    private final static I2C.Port i2cPort = I2C.Port.kOnboard;
    static ColorSensorV3 sensor = new ColorSensorV3(i2cPort);

    public static double[] getColor(){
        double[] rgb = new double[]{sensor.getRed(),sensor.getGreen(),sensor.getBlue()};
        System.out.println(rgb2hsv(rgb));
        return rgb2hsv(rgb);
    }

    //HSV!
    public static double[] rgb2hsv(double[] rgb) {

        double[] hsv = new double[3];

        double max = Math.max(Math.max(rgb[0], rgb[1]), rgb[2]);
        double min = Math.min(Math.min(rgb[0], rgb[1]), rgb[2]);
        double delta = max - min;

        if (delta == 0) {
            hsv[0] = 360;
            hsv[1] = 0;
            hsv[2] = max;
            return hsv;
        }

        if (max == rgb[0]) {
            hsv[0] = (rgb[1] - rgb[2]) / delta % 6;
        } else if (max == rgb[1]) {
            hsv[1] = (rgb[2] - rgb[0]) / delta + 2;
        } else {
            hsv[2] = (rgb[0] - rgb[1]) / delta + 4;
        }
        hsv[0] *= 60;

        if (max == 0) {
            hsv[1] = 0;
        } else {
            hsv[1] = delta / max;
        }

        hsv[2] = max;

        return hsv;
    }

    public static boolean hasNote(){
        return true;
    }
}
