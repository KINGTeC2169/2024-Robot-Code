package frc.robot.subsystems;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private CANcoder absoluteEncoder;

    private final RelativeEncoder turnEncoder;

    private double wantedSpeed;

    private PIDController drivePID;
    private PIDController turningPID;
    private CANcoderConfiguration AbsoluteConfig;

    final VoltageOut request = new VoltageOut(0);

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed) {

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        //Configuration for CANCoder
        AbsoluteConfig = new CANcoderConfiguration();
        AbsoluteConfig.MagnetSensor.MagnetOffset = Units.radiansToDegrees(absoluteOffset);
        absoluteEncoder = new CANcoder(canCoderID);
        absoluteEncoder.getConfigurator().apply(AbsoluteConfig);

        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        var driveConfig = new Slot0Configs();
        driveConfig.kP = 0.1;
        driveMotor.getConfigurator().apply(driveConfig);

        resetEncoders();

    }

    public double getDrivePosition() {
        return -driveMotor.getRotorPosition().getValueAsDouble() * (0.32 / 13824);
    }

    /**Returns position of turn encoder in radians. Counterclockwise is positive, accumulates. */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getTurnPosition());
    }

    
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getTurnCurrent() {
        return turnMotor.getOutputCurrent();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.setPosition(0);
        turnEncoder.setPosition(getAbsoluteTurnPosition());
        System.out.println("RESETTING ENCODERS \nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        if(state.speedMetersPerSecond > 0) {
        wantedSpeed = (((state.speedMetersPerSecond / maxSpeed) * 0.94) + 0.06);
        // I LOVE ALIVEBAND  
        driveMotor.set(((state.speedMetersPerSecond / maxSpeed) * 0.94) + 0.06);

        } else {
        wantedSpeed = (((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        // I LOVE ALIVEBAND  
        driveMotor.set(((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        }
        
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));

    }
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(-state.speedMetersPerSecond / maxSpeed);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public double getError() {
        return driveMotor.getClosedLoopError().getValueAsDouble();
    }

    public double getWantedSpeed() {
        return wantedSpeed;
    }

    public void stop() {
        wantedSpeed = 0;
        driveMotor.set(0);
        turnMotor.set(0);
    }


    //EXPERIMENTAL METHODS:

    /**
     * Uses a PID to set drive velocity to 0
     */
    public void semiAutoStop() {
        driveMotor.set(drivePID.calculate(getDriveVelocity(), 0));
        turnMotor.set(0);
    }

    public void fullStop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
    /**
     * Sets wheels to X formation
     */
    public void activeStop(int direction) {
        System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(0);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

}   