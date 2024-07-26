package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class SelfTest extends Command {
    private SwerveSubsystem swerveSubsystem;
    private Shooter shooter;
    private Intake intake;
    private Arm arm;
    
    public SelfTest(SwerveSubsystem swerveSubsystem, Shooter shooter, Intake intake, Arm arm){
        this.swerveSubsystem = swerveSubsystem;
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;
        addRequirements(swerveSubsystem, shooter, intake, arm);
    }

    @Override
    public void execute(){
        System.out.println("-------------------------------------------------------------------------");
        System.out.println("-----------------------Starting Merlin's Self Test-----------------------");
        System.out.println("-------------------------------------------------------------------------\n\n\n");

        System.out.println("Testing Front Left Swerve Module...");
        swerveSubsystem.getFrontLeft().testTurnMotor();
        swerveSubsystem.getFrontLeft().testDriveMotor();
        System.out.println();

        System.out.println("Testing Front Right Swerve Module...");
        swerveSubsystem.getFrontRight().testTurnMotor();
        swerveSubsystem.getFrontRight().testDriveMotor();
        System.out.println();

        System.out.println("Testing Back Left Swerve Module...");
        swerveSubsystem.getBackLeft().testTurnMotor();
        swerveSubsystem.getBackLeft().testDriveMotor();
        System.out.println();

        System.out.println("Testing Back Right Swerve Module...");
        swerveSubsystem.getBackRight().testTurnMotor();
        swerveSubsystem.getBackRight().testDriveMotor();
        System.out.println();

        System.out.println("Testing Swerve Subsystem...");
        swerveSubsystem.resetEncoders();
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(1, 0, 0 , Pigeon.getRotation2d());
        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setModuleStates(swerveModuleStates);
        Timer.delay(3);
        chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, Pigeon.getRotation2d());
        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setModuleStates(swerveModuleStates);
        Timer.delay(1);
        chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 1, 0, Pigeon.getRotation2d());
        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setModuleStates(swerveModuleStates);
        Timer.delay(3);
        chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, Pigeon.getRotation2d());
        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setModuleStates(swerveModuleStates);
        Timer.delay(1);
        chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 1, Pigeon.getRotation2d());
        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setModuleStates(swerveModuleStates);
        Timer.delay(3);
        chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, Pigeon.getRotation2d());
        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setModuleStates(swerveModuleStates);
        System.out.println("SWERVE SUBSYSTEM TEST COMPLETE\n");
        
        System.out.println("Testing Arm Subsystem...");
        System.out.println("Testing Amp Position...");
        while (!arm.isReady()) arm.setShootPos(Constants.Positions.amp);
        System.out.println("Testing Podium Position...");
        while (!arm.isReady()) arm.setShootPos(Constants.Positions.podium);
        System.out.println("Testing Subwoofer Position...");
        while (!arm.isReady()) arm.setShootPos(Constants.Positions.subwoofer);
        System.out.println("Testing Rest Position...");
        arm.setRest(false);
        System.out.println("ALL ARM COMMANDS EXECUTED\n");

        System.out.println("Testing Shooter Subsystem...");
        shooter.setRPM(5000);
        Timer.delay(2);
        shooter.setRPM(0);
        System.out.println("SHOOTER SUBSYSTEM TEST COMPLETE\n");

        System.out.println("Testing Intake Subsystem and Beam Break Sensor...");
        System.out.println("Please Feed Note...");
        intake.inTake();
        while (!NoteManager.hasNote()) {
            if (NoteManager.hasNote()){
                intake.stopTake();
                intake.outTake();
                Timer.delay(0.02);
                intake.stopTake();
            }
        }
        System.out.println("INTAKE SUBSYSTEM AND BEAM BREAK SENSOR TEST COMPLETE\n");
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("-------------------------------------------------------------------------");
        System.out.println("----------------------------Self Test Complete---------------------------");
        System.out.println("-------------------------------------------------------------------------\n\n\n"); 
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}