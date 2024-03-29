// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Ports {
        
        public static final int leftStick = 0;
        public static final int rightStick = 1;
        public static final int controller = 2;
        public static final int buttons = 3;

        public static final int pigeon = 14;

        //Swervedrive CAN ids

        public static final int frontLeftDrive = 3;
        public static final int frontLeftTurn = 2;
        public static final int frontLeftAbsolute = 10;
        public static final int frontRightDrive = 5;
        public static final int frontRightTurn = 4;
        public static final int frontRightAbsolute = 11;
        public static final int backLeftDrive = 7;
        public static final int backLeftTurn = 6;
        public static final int backLeftAbsolute = 12;
        public static final int backRightDrive = 9;
        public static final int backRightTurn = 8;
        public static final int backRightAbsolute = 13;
        
        //DIO ports
        public static final int beamBreak = 0;
        public static final int armEncoder = 8;

        //Shooter CAN ids
        public static final int shooterTop = 17;
        public static final int shooterBot = 18;

        //Intake CAN id
        public static final int intake = 19; 

        //Arm CAN ids
        public static final int leftArm = 15;
        public static final int rightArm = 16;

        public static final Port arduino = SerialPort.Port.kUSB1; //Arduino USB port
        
    }

    public static final class AutoAngles{

        public static final double top = 0.3;
        
           



    }

    public static final class ArmConstants {
        public static final double armEncoderOffset =  0.58822 + 0.292;//0.2294932807 - 0.25 + 0.58822 - 0.292;
        //public static final double armEncoderOffset =  0.2294932807 - 0.25 + (1-0.58822) - 0.292;
        public static final double shooterOffset = 0.4103968962; // (90-66.486)*0.0174533 RAD
        public static final double armOffset = 0.1504823526;//   RAD
        public static final double distance = 2.06841667; //ft Distance from hex shaft to point of shot 24.821in

        public static final double armGearBox = 201.6; //48 * (84/20)


    }

    public static final class ModuleConstants {
        //public static final double maxNeoSpeed = 3.68808;
        public static final double maxSpeed = 4.96824;
        public static final double maxNeoRadPerSec = 2 * 2 * Math.PI;
        public static final double wheelDiameter = 0.1016;//Units.inchesToMeters(4.0);
        public static final double driveGearRatio = 1 / 6.12;
        public static final double turnGearRatio = 1 / 12.8;
        public static final double driveEncoderToMeter = driveGearRatio * Math.PI * wheelDiameter;
        public static final double turnEncoderToRadian = turnGearRatio * 2 * Math.PI;
        public static final double driveEncoderRPMToMeterPerSec = driveEncoderToMeter / 60;
        public static final double turnEncoderRPMToRadPerSec = turnEncoderToRadian / 60;


        public static final ShuffleboardTab tab = Shuffleboard.getTab("Swerve Module");

        public static double PDrive = tab.add("Drive P", 0.1).getEntry().getDouble(0.1);

        public static double PTurn = 0.31;
    }

    public static final class DriveConstants {
        //These will need to be in meters
        public static final double rightLeftWheels = Units.inchesToMeters(23);
        public static final double frontBackWheels = Units.inchesToMeters(23);


        public static final double FRabsoluteOffset = -0.470931150019169; //2.672189131379128; 
        public static final double FLabsoluteOffset = -2.178248316049576; //0.958736054599285; 
        public static final double BRabsoluteOffset = 0.572173677384853; //0.513882525265217; 
        public static final double BLabsoluteOffset = 0.628930851817131; //2.517257384955883; 


        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(frontBackWheels / 2, rightLeftWheels / 2),//Front-Left
                new Translation2d(frontBackWheels / 2, -rightLeftWheels / 2),//Front-Right
                new Translation2d(-frontBackWheels / 2, rightLeftWheels / 2),//Back-Left
                new Translation2d(-frontBackWheels / 2, -rightLeftWheels / 2));//Back-Right
    }

    public static final class DeviceID {
        public static final int shooterTop = 16;
        public static final int shooterBot = 14;
        public static final int intake = 15;
        public static final int leftArm = 17;
        public static final int rightArm = 18;
        
    }
    public static final class Vision {

        //7 3/4 from edge
        public static final double shootRPM = 5000; //Do not change. Currently about 5000 TODO: Make it adjustable
        public static final double tagHeight = 5.5; //ft
        public static final double mountedHeight = 1.0; //ft
        public static final double mountedAngle = 45; //angle deg
        public static final double toShaftX = -0.1146; //ft
        public static final double toShaftY = 0.28125; //ft
        public static final double launchSpeed = 50; //ft/sec
        public static final double gravity = -32.19;

    }

    public static final class Positions{
        public static final double rest = 0.292;
        public static final double subwoofer = 0.3517;
        public static final double sideSubwoofer = 0.36334;
        public static final double podium = 0.3877;
        public static final double amp = 0.5321;
    }

    
}