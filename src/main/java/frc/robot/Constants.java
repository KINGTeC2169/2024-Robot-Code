// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.SerialPort;

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
        public static final int controller = 0;
        public static final int leftStick = 0;
        public static final int rightStick = 1;

        //public static final SerialPort arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
        public static final int beamBreak = 1;
        public static final int rightArmEncoder = 21;
        public static final int leftArmEncoder = 22;
        public static final int pigeon = 14;

        //Swervedrive ports
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
        

        
    }

    public static final class Motors {
        public static final int TalonFXCPR = 2048;
        public static final int TalonSRXCPR = 8192;

        public static final double armGearBox = 36;
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

        public static double PTurn = 0.31;
        public static double PDrive = 10;
    }

    public static final class DriveConstants {
        //These will need to be in meters
        public static final double rightLeftWheels = Units.inchesToMeters(23);
        public static final double frontBackWheels = Units.inchesToMeters(23);

        public static final double FRabsoluteOffset = -0.464795239269733; //-0.464795239269733; 
        public static final double FLabsoluteOffset = 0.857493527233601; //-0.862095460295677; 
        public static final double BRabsoluteOffset = 0.513882525265217; //0.513882525265217; 
        public static final double BLabsoluteOffset = -2.520325340330601; //2.517257384955883; 

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(frontBackWheels / 2, rightLeftWheels / 2),//Front-Left
                new Translation2d(frontBackWheels / 2, -rightLeftWheels / 2),//Front-Right
                new Translation2d(-frontBackWheels / 2, rightLeftWheels / 2),//Back-Left
                new Translation2d(-frontBackWheels / 2, -rightLeftWheels / 2));//Back-Right
    }

    public static final class DeviceID {
        public static final int shooterTop = 1;
        public static final int shooterBot = 14;
        public static final int intake = 15;
        public static final int leftArm = 16;
        public static final int rightArm = 18;
        
    }

    public static final class Vision {
        public static final double shootRPM = 300; //Do not change. Currently about 5000 TODO: Make it adjustable
        public static final double tagHeight = 5.5; //ft
        public static final double mountedHeight = 1.5; //ft
        public static final double mountedAngle = 45; //angle deg
        public static final double launchSpeed = 75; //ft/sec
        public static final double gravity = -32.19;
    }
}