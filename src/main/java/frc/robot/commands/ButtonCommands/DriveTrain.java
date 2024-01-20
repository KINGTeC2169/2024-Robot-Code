package frc.robot.commands.ButtonCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTrain extends Command{
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> reset;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public DriveTrain(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, 
        Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> reset){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.reset = reset;
        this.xLimiter = new SlewRateLimiter(5);
        this.yLimiter = new SlewRateLimiter(5);
        this.turningLimiter = new SlewRateLimiter(5);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        xSpeed = xLimiter.calculate(xSpeed) *  Constants.ModuleConstants.maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) *  Constants.ModuleConstants.maxSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * ModuleConstants.maxNeoRadPerSec;

        if (reset.get()){
            swerveSubsystem.resetEncoders();
            swerveSubsystem.zeroHeading();
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);

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
