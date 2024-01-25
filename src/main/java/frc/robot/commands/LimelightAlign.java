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
    private double tagHeightFt = 5.6;
    private double baseAngleDeg;
    private double baseHeightFt;
    private double totalAngleDeg;
    private double totalDistanceFt;


    public LimelightAlign(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        turnController = new PIDController(0.5,0,0);
        baseAngleDeg = 10;
        baseHeightFt = 1.5;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(LimelightTable.getTV()) chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, turnController.calculate(LimelightTable.getTX(), 0), swerveSubsystem.getRotation2d());
        else chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        totalAngleDeg = baseAngleDeg+LimelightTable.getTA();
        totalDistanceFt = (tagHeightFt-baseHeightFt)*Math.tan(totalAngleDeg); //Horizontal distance in feet
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
