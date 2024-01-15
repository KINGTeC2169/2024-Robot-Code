package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    //private final RelativeEncoder driveEncoder;

    private double wantedSpeed;

    private PIDController drivePID;
    private PIDController turningPID;
    private CANcoderConfiguration config = new CANcoderConfiguration();

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed) {
        
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        //driveMotor.configFactoryDefault(); commented out in 2022 offseason.
        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        config.MagnetSensor.MagnetOffset = Units.radiansToDegrees(absoluteOffset);
        //config.MagnetSensor.sensorCoefficient = 2 * Math.PI/4096.0;
        //config.MagnetSensor.UnitString = "rad";
        //config.MagnetSensor.SensorTimeBase = SensorTimeBase.PerSecond;
        //config.MagnetSensor.SensorDirection = isCancoderReversed;

        
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        absoluteEncoder = new CANcoder(canCoderID);
        absoluteEncoder.getConfigurator().apply(config);


        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);
        //driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        /* */
        //driveEncoder.setPositionConversionFactor(driveEncoderToMeter);
        //driveEncoder.setVelocityConversionFactor(driveEncoderRPMToMeterPerSec);
        //driveEncoder = driveMotor.getEncoder(); !!!! NOT UPDATED

        /* */

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, 0, 0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        //driveMotor.kP(0, 0.1); !!!! NOT UPDATED

        
        //drivePID = new PIDController(PDrive, 0, 0);


        resetEncoders();

    }

    public double getDrivePosition() {
        //return driveMotor.getSelectedSensorPosition() * driveEncoderToMeter;
        //return -driveMotor.getSelectedSensorPosition() * (0.32 / 13824); NOT UPDATED
        return 0;
    }

    /**Returns position of turn encoder in radians. Counterclockwise is positive, accumulates. */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }
    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(getTurnPosition());
        return Rotation2d.fromRadians(getTurnPosition());
    }

    
    public double getDriveVelocity() {
        //return driveMotor.getSelectedSensorVelocity() * driveEncoderRPMToMeterPerSec;
        //return driveMotor.getSelectedSensorVelocity(); NOT UPDATED
        return 0;
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
        //>return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteTurnPosition() {
        //return absoluteEncoder.getAbsolutePosition(); Not updated
        return 0;
    }

    public double getDriveCurrent() {
        //return driveMotor.getSupplyCurrent(); Not Updated
        return 0;
    }

    public double getTurnCurrent() {
        return turnMotor.getOutputCurrent();
    }

    public void resetEncoders() {
        //driveMotor.setSelectedSensorPosition(0); not updated
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
            //TODO: this causes an aggresive stop, need to test with more weight on bot
            semiAutoStop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / 5); // 5 = temp maxNeoSpeed
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        //driveMotor.set(ControlMode.PercentOutput, -((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        //driveMotor.set(ControlMode.PercentOutput, -state.speedMetersPerSecond / maxSpeed);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public double getError() {
        //!return driveMotor.getClosedLoopError();
        return 0;
    }

    public double getWantedSpeed() {
        return wantedSpeed;
    }

    public void stop() {
        wantedSpeed = 0;
        //driveMotor.set(ControlMode.Velocity, 0);
        //!driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(0);
    }


    //EXPERIMENTAL METHODS:

    /**
     * Uses a PID to set drive velocity to 0
     */
    public void semiAutoStop() {
        //!driveMotor.set(ControlMode.PercentOutput, drivePID.calculate(getDriveVelocity(), 0));
        turnMotor.set(0);
    }

    public void fullStop() {
        //!driveMotor.set(ControlMode.Velocity, 0);
        turnMotor.set(0);
    }
    /**
     * Sets wheels to X formation
     */
    public void activeStop(int direction) {
        System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        //!driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

}