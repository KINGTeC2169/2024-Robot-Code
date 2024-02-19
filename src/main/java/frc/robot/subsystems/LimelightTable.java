package frc.robot.subsystems;

import static java.lang.Math.*;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Vision;



public class LimelightTable {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
    private HttpCamera limelightCam = new HttpCamera("limelight", "http://10.21.69.33:5800");

    //This constructor adds data to the Limelight Shuffleboard tab
    public LimelightTable(){
        tab.add("Limelight", limelightCam);
        tab.addNumber("TX", () -> getTX());
        tab.addNumber("TY", () -> getTY());
        tab.addNumber("TA", () -> getTA());
        tab.addNumber("TS", () -> getTS());
        tab.addBoolean("canSeeTag", () -> getTV());
        tab.addNumber("TagID", () -> getID());
        tab.addNumber("Distance", () -> getDistance());
        tab.addNumber("Aim shot", () -> aimShot());
        tab.addDouble("Angle", () -> (Vision.mountedAngle + LimelightTable.getTY()));

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
        return table.getEntry("tid").getDouble(0);
    }

    public static boolean getTV(){
        return table.getEntry("tv").getDouble(0.0) > 0;
    }

    public static double getAngle(){
        return Vision.mountedAngle+LimelightTable.getTY();
    }

    public static double getHeightDif(){
        return Vision.tagHeight-Vision.mountedHeight;
    }
    
    public static double getDistance(){
        return getHeightDif() * Math.tan(getAngle()); //Horizontal distance in feet
    }

    /**
     * Finds the angle necessary to hit a point
     * @param target target[0] is distance, target[1] is height
     * @return shooting angle to hit a point
     */
    public static double getShootingAngle(double[] target){
        return atan((pow(Vision.launchSpeed, 2) * (-target[0] + sqrt((pow(target[0], 2)) - (4 * ((Vision.gravity * ((pow(target[0], 2)))) / (2 * (pow(Vision.launchSpeed, 2)))) * (((Vision.gravity * ((pow(target[0], 2)))) / (2 * (pow(Vision.launchSpeed, 2)))) - target[1] + Vision.mountedHeight)))) / (Vision.gravity * (pow(target[0], 2)))))*180/PI;
    }

    //Averages the top and bottom of the speaker to get an aiming distance
    public static double aimShot(){
        double distance = getDistance();
        double finalAim = getShootingAngle(new double[]{distance, 6.90625});
        //SmartDashboard.putNumber("Shot", aimShot());
        return finalAim;
    }
}