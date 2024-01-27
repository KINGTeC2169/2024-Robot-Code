package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightTable;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightAlign extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ChassisSpeeds chassisSpeeds;
    private PIDController turnController;


    public LimelightAlign(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        turnController = new PIDController(0.5,0,0);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(LimelightTable.getTV()) chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, turnController.calculate(LimelightTable.getTX(), 0), swerveSubsystem.getRotation2d());
        else chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
