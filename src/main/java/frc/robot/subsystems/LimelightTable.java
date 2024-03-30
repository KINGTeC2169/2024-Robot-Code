package frc.robot.subsystems;

import static java.lang.Math.*;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Vision;



public class LimelightTable {
    private static double finalAim;
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
    private HttpCamera limelightCam = new HttpCamera("limelight", "http://10.21.69.33:5800");

    //This constructor adds data to the Limelight Shuffleboard tab
    public LimelightTable(){
        
        tab.addNumber("TX", () -> getTX()).withPosition(0, 0);
        tab.addNumber("TY", () -> getTY()).withPosition(1, 0);
        tab.addNumber("TA", () -> getTA()).withPosition(0, 1);
        tab.addNumber("TS", () -> getTS()).withPosition(1, 1);
        tab.addBoolean("canSeeTag", () -> getTV()).withPosition(0, 2);
        tab.addNumber("TagID", () -> getID()).withPosition(1, 2);

        tab.add("Limelight", limelightCam).withWidget(BuiltInWidgets.kCameraStream).withSize(3, 3).withPosition(2, 0).withProperties(Map.of("Show Controls", false));

        tab.addNumber("Distance", () -> getDistance()).withPosition(5, 0);
        tab.addNumber("Aim shot", () -> aimShot()).withPosition(6, 0);
        tab.addDouble("Angle", () -> (getAngle())).withPosition(5, 1);
        tab.addDouble("HeightDif", () -> getHeightDif()).withPosition(6, 1);
        tab.addNumber("Aim shot1", () -> finalAim).withPosition(6, 0);
        tab.addNumber("Angle shot", () -> Arm.aimToArm(LimelightTable.aimShot())).withPosition(7,0);
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
        return (getHeightDif()/Math.sin(getAngle()*0.0174533)); //Horizontal distance in feet
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
        double aim = getShootingAngle(new double[]{distance, 6.90625-Vision.mountedHeight});
        //double aim = getShootingAngle(new double[]{distance-Arm.predictArmPosition(1)[0], 6.90625-Vision.mountedHeight-Arm.predictArmPosition(1)[1]});
        for(int i = 0; i < 8; i++){
            aim = getShootingAngle(new double[]{distance-Arm.predictArmPosition(Arm.aimToArm(aim))[0], 6.90625-Vision.mountedHeight-Arm.predictArmPosition(Arm.aimToArm(aim))[1]});
        }
        finalAim = aim;
        return finalAim;
    }
}