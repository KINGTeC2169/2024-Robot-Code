package frc.robot.commands.ButtonCommands.ShootingPos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimelightTable;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightAlign extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ChassisSpeeds chassisSpeeds;
    private PIDController turnController;

    private Arm arm;
    private Shooter shooter;

    public LimelightAlign(SwerveSubsystem swerveSubsystem, Arm arm, Shooter shooter){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        this.arm = arm;
        addRequirements(arm);
        this.shooter = shooter;
        addRequirements(shooter);
        turnController = new PIDController(0.075,0,0);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(LimelightTable.getTV()) chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, -turnController.calculate(LimelightTable.getTX(), 0), swerveSubsystem.getRotation2d());
        else chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        arm.setAim(LimelightTable.aimShot());
        //shooter.setRPM(Constants.Vision.shootRPM);
    }

    @Override
    public void end(boolean interrupted) {
        arm.activeStop();
    }

    @Override
    public boolean isFinished() {
        return arm.isReady() && shooter.shooterReady();
    }
}
