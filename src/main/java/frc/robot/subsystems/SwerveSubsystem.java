package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Ports;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


public class SwerveSubsystem extends SubsystemBase {

    //Creates an instance of SwerveModule for each module on the robot
    private SwerveModule frontLeft = new SwerveModule(
    Ports.frontLeftDrive,
    Ports.frontLeftTurn, 
    false, false,
    Ports.frontLeftAbsolute,
    DriveConstants.FLabsoluteOffset,
    false);

    private SwerveModule frontRight = new SwerveModule(
    Ports.frontRightDrive,
    Ports.frontRightTurn, 
    false, false,
    Ports.frontRightAbsolute,
    DriveConstants.FRabsoluteOffset,
    false);

    private SwerveModule backLeft = new SwerveModule(
    Ports.backLeftDrive,
    Ports.backLeftTurn, 
    false, false,
    Ports.backLeftAbsolute,
    DriveConstants.BLabsoluteOffset,
    false);

    private SwerveModule backRight = new SwerveModule(
    Ports.backRightDrive,
    Ports.backRightTurn, 
    true, false,
    Ports.backRightAbsolute,
    DriveConstants.BRabsoluteOffset,
    false);

    public SwerveDriveKinematics kinematics = DriveConstants.DRIVE_KINEMATICS;
    private final SwerveDriveOdometry odometer;
    public Field2d field = new Field2d();
    public PathPlannerTrajectory trajectory;

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    private GenericEntry fastSpeed = tab.add("Fast Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(6, 3).getEntry();
    private GenericEntry mediumSpeed = tab.add("Medium Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(4, 3).getEntry();
    private GenericEntry slowSpeed = tab.add("Slow Speed", 0.2).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(2, 3).getEntry();

    public SwerveSubsystem() {

        Pigeon.configure();

        odometer = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
        
        //SmartDashboard.putData("Field", field);
        
        ShuffleboardLayout driveCurrents = tab.getLayout("Drive Currents", BuiltInLayouts.kGrid).withSize(2, 2).withProperties(Map.of("Number of rows", 2)).withPosition(0, 0);
        ShuffleboardLayout turnCurrents = tab.getLayout("Turn Currents", BuiltInLayouts.kGrid).withSize(2, 2).withProperties(Map.of("Number of rows", 2)).withPosition(0, 2);

        driveCurrents.addDouble("Front Left", () -> frontLeft.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));
        driveCurrents.addDouble("Front Right", () -> frontRight.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));
        driveCurrents.addDouble("Back Left", () -> backLeft.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));
        driveCurrents.addDouble("Back Right", () -> backRight.getDriveCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));

        turnCurrents.addDouble("Front Left", () -> frontLeft.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));
        turnCurrents.addDouble("Front Right", () -> frontRight.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));
        turnCurrents.addDouble("Back Left", () -> backLeft.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));
        turnCurrents.addDouble("Back Right", () -> backRight.getTurnCurrent()).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Orientation", "VERTICAL", "Max", 100));

        tab.addDouble("Front Left Absolute", () -> frontLeft.getAbsoluteTurnPosition());
        tab.addDouble("Front Right Absolute", () -> frontRight.getAbsoluteTurnPosition());
        tab.addDouble("Back Left Absolute", () -> backLeft.getAbsoluteTurnPosition());
        tab.addDouble("Back Right Absolute", () -> backRight.getAbsoluteTurnPosition());
        

        tab.addDouble("Robot Heading", () -> getHeading()).withWidget(BuiltInWidgets.kGyro).withSize(3, 3).withPosition(7, 0);

        tab.add(field).withPosition(2, 0).withSize(5, 3);

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.01, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.01, 0.0, 0.0), // Rotation PID constants
                    ModuleConstants.maxSpeed, // Max module speed, in m/s
                    0.291, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

        //Creates a new thread, which sleeps and then zeros out the gyro
        //Uses a new thread so that it doesn't pause all other code running
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                zeroHeading();
                resetEncoders();
            } catch (Exception e) {
            }
        }).start();
    }

    /**Returns the field. */
    public Field2d getField() {
        return field;
    }

    public void setTrajectory(PathPlannerTrajectory trajectory){
        this.trajectory = trajectory;
    }

    /**Returns the fast speed, which is adjustable via the slider on shuffleboard.*/
    public double getFastSpeed() {
        return fastSpeed.getDouble(0.8);
    }

    /**Returns the medium speed, which is adjustable via the slider on shuffleboard.*/
    public double getMediumSpeed() {
        return mediumSpeed.getDouble(0.65);
    }

    /**Returns the slow speed, which is adjustable via the slider on shuffleboard.*/
    public double getSlowSpeed() {
        return slowSpeed.getDouble(0.3);
    }
    
    /**Returns the Module positions of the 4 swerve modules in the order frontLeft, frontRight, backLeft, backRight.*/
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
        };
        
    }

    /**Resets the Pigeon.*/
    public void zeroHeading() {
        System.out.println("Zeroing gyro \n.\n.\n.\n.\n.\n.\n.");
        Pigeon.reset();
    }

    /**Returns the heading of the pigeon.*/
    public double getHeading() {
        //return Math.IEEEremainder(NavX.getAngle(), 360);
        return Pigeon.getAngle() % 360;
    }

    /**Returns the rotation2d from the pigeon.*/

    public Rotation2d getRotation2d() {
        return Pigeon.getRotation2d();
        //return Rotation2d.fromDegrees(-getHeading());
    }


    /**Returns chassis speeds that are relative to the robot's front.*/
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates()[0], 
                                          getModuleStates()[1], 
                                          getModuleStates()[2], 
                                          getModuleStates()[3]);
    }

    /**Sets swerve module states relative to the robot's front. */
    public void driveRobotRelative(ChassisSpeeds speeds){
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**Returns the odometer's position in the form of a Pose2d. */
    public Pose2d getPose() {
        //return NavX.getPose();
        return odometer.getPoseMeters();
    }

    /**Resets the position of the odometer using the rotation2d of the pigeon, the module positions, and a pose2d. */
    public void resetOdometry(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                odometer.resetPosition(getRotation2d().plus(new Rotation2d(180)), getModulePositions(), pose);
            }else{
                odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
            }
    }

    /**Resets the encoders of the 4 swerve modules. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());

        //field.setRobotPose(odometer.getPoseMeters());
        field.setRobotPose(odometer.getPoseMeters().getX(), odometer.getPoseMeters().getY(), odometer.getPoseMeters().getRotation());

        SmartDashboard.putNumber("X", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometer.getPoseMeters().getY());
        SmartDashboard.putNumber("Pose angle", odometer.getPoseMeters().getRotation().getDegrees());
        
        //SmartDashboard.putData("Field", field);
    
        /*
        SmartDashboard.putNumber("Front Left", frontLeft.getTurnPosition());
        SmartDashboard.putNumber("Front Right", frontRight.getTurnPosition());
        SmartDashboard.putNumber("Back Left", backLeft.getTurnPosition());
        SmartDashboard.putNumber("Back Right", backRight.getTurnPosition());
        SmartDashboard.putNumber("Wanted Speed", backRight.getWantedSpeed());
        SmartDashboard.putNumber("Back Right speed", backRight.getDriveVelocity());
        SmartDashboard.putNumber("Back Left speed", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Front Right speed", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("Front left speed", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Error", backLeft.getError());
        SmartDashboard.putNumber("Error", Math.abs(backRight.getWantedSpeed() - backRight.getDrivePosition()));
        */
    }

    /**Stops all 4 swerve modules. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

     /**Takes an array of SwerveModuleStates and sets each SwerveModule to its respective state */
     public void setModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.maxSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);

    }

    /**Sets modules states for autos only. */
    public void setAutoModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.maxSpeed);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);

    }

    /**Gets all 4 swerve module states. */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()};
    }

    /**Stops the wheels. Untested. Not used.*/
    public void fullStop() {
        frontLeft.fullStop();
        frontRight.fullStop();
        backLeft.fullStop();
        backRight.fullStop();
    }

    /**Puts wheels in 'X' position and sets driving to a velocity-PID loop set at 0m/s */
    public void setActiveStop() {
        System.out.println("1\n1\n1\n1\n1\n1\n1\n1");
        frontLeft.activeStop(-1);
        frontRight.activeStop(1);
        backLeft.activeStop(1);
        backRight.activeStop(-1);
    }

}