package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Pigeon {
    
    private static Pigeon2 pigeon = new Pigeon2(Constants.Ports.pigeon);
    private ShuffleboardTab tab = Shuffleboard.getTab("Pigeon");

    public Pigeon(){
        tab.addDouble("Yaw", () -> getYaw());
        tab.addDouble("Pitch", () -> getPitch());
        tab.addDouble("Roll", () -> getRoll());
        tab.addDouble("Angle", () -> getAngle());
    }

    public static double getX(){
        return pigeon.getYaw().getValueAsDouble();
    }

    public static double getY(){
        return pigeon.getPitch().getValueAsDouble();
    }

    public static double getRoll(){
        return pigeon.getRoll().getValueAsDouble();
    }

    public static double getPitch(){
        return pigeon.getPitch().getValueAsDouble();
    }

    public static double getYaw(){
        return pigeon.getYaw().getValueAsDouble();
    }

    public static Rotation2d getRotation2d(){
        return pigeon.getRotation2d();
    }

    public static double getAngle(){
        return pigeon.getAngle();
    }

    public static void reset(){
        pigeon.reset();
    }

    public static void configure(){
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }
}
