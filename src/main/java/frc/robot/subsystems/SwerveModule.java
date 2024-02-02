package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Conversions;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private CANcoder absoluteEncoder;

    private RelativeEncoder turnEncoder;

    private double wantedSpeed;

    private PIDController drivePID;
    private PIDController turningPID;
    private MagnetSensorConfigs AbsoluteConfig;
    private TalonFXConfiguration driveConfig;

    private VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int driveMotorID, int turnMotorID, InvertedValue driveMotorDirection, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                SensorDirectionValue cancoderDirection) {

        //Confgure CANCoder
        absoluteEncoder = new CANcoder(canCoderID);
        AbsoluteConfig = new MagnetSensorConfigs();
        absoluteEncoder.getConfigurator().apply(AbsoluteConfig.withMagnetOffset(absoluteOffset).withSensorDirection(cancoderDirection));

        //Configure drive motor
        driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = driveMotorDirection;
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.getConfigurator().apply(driveConfig);
        
        //Configure turn motor
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);
        turnMotor.setInverted(turnMotorReversed);

        drivePID = new PIDController(PDrive, 0, 0);

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    /**Returns position of turn encoder in rotations. Counterclockwise is positive, accumulates. */
     public double getTurnPosition() {
         return turnEncoder.getPosition();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getTurnPosition());
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteTurnPosition() {
        return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getTurnCurrent() {
        return turnMotor.getOutputCurrent();
    }

    //Resets the turn encoder to the absolute turn position and the drive motor to 0
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turnEncoder.setPosition(getAbsoluteTurnPosition());
        System.out.println("RESETTING ENCODERS \nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(Conversions.RPSToMPS(getDriveVelocity(), Constants.ModuleConstants.wheelDiameter) , 
                                     new Rotation2d(getTurnPosition()));
    }
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(Conversions.rotationsToMeters(getDrivePosition(), Constants.ModuleConstants.wheelDiameter), 
                                        getRotation2d());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        driveVelocity.Velocity = Conversions.MPSToRPS(state.speedMetersPerSecond, Constants.ModuleConstants.wheelDiameter);
        driveVelocity.FeedForward = drivePID.calculate(state.speedMetersPerSecond);
        driveMotor.setControl(driveVelocity);
        
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));

    }
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(-state.speedMetersPerSecond / maxSpeed);
        System.out.println(-state.speedMetersPerSecond / maxSpeed);
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

    public void fullStop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void activeStop(int direction) {
        System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(0);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }
}   