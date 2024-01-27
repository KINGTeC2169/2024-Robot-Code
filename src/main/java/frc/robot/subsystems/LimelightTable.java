package frc.robot.subsystems;

import static java.lang.Math.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Vision;



public class LimelightTable {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

    //This constructor adds data to the Limelight Shuffleboard tab
    public LimelightTable(){
        tab.addNumber("TX", () -> getTX());
        tab.addNumber("TY", () -> getTY());
        tab.addNumber("TA", () -> getTA());
        tab.addNumber("TS", () -> getTS());
        tab.addBoolean("canSeeTag", () -> getTV());
        tab.addNumber("TagID", () -> getID());
    }

    public static double getTX(){
        return table.getEntry("tx").getDouble(0);
    }

    public static double getTY(){
        return table.getEntry("ty").getDouble(0);
    }

    public static double getTA(){
        return table.getEntry("ta").getDouble(0);
    }

    public static double getTS(){
        return table.getEntry("ts").getDouble(0);
    }

    public static double getID(){
        return table.getEntry("tagID").getDouble(0);
    }

    public static boolean getTV(){
        return table.getEntry("tv").getDouble(0.0) > 0;
    }
    
    public static double getDistance(){
        double totalAngleDeg = Vision.mountedAngle+LimelightTable.getTY();
        return (Vision.tagHeight-Vision.mountedHeight)*Math.tan(totalAngleDeg); //Horizontal distance in feet
    }

    /**
     * Finds the angle necessary to hit a point
     * @param target target[0] is distance, target[1] is height
     * @return shooting angle to hit a point
     */
    public static double getShootingAngle(double[] target){
        double a = 1, offset = 1, digits = 1;
        while(abs(offset) > 0.00001) {
            offset = target[1]-(Vision.mountedHeight+(Vision.launchSpeed*sin(a*0.0174533))*((target[0])/(Vision.launchSpeed*cos(a*0.0174533)))+(-16.09*pow(((target[0])/(Vision.launchSpeed*cos(a*0.0174533))),2)));
            if(abs(offset) < 1/digits) digits*=10;
            if(offset > 0) a += 1/digits;
            else a -= 1/digits;
        }
        return a;
    }

    //Averages the top and bottom of the speaker to get an aiming distance
    public static double aimShot(){
        double distance = getDistance();
        double max = getShootingAngle(new double[]{distance-1.5, 6.90625});
        double min = getShootingAngle(new double[]{distance,6.5});
        return (max+min)/2;
    }
}