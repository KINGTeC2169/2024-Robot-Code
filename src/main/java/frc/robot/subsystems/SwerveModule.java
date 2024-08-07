
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private CANcoder absoluteEncoder;

    private final RelativeEncoder turnEncoder;

    private double wantedSpeed;
    private PIDController turningPID;

    private CANcoderConfiguration config;
    

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
                boolean turnMotorReversed, int canCoderID, double absoluteOffset, 
                boolean isCancoderReversed) {

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration().Slot0.withKP(Constants.ModuleConstants.PDrive));
        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        turnMotor.setIdleMode(IdleMode.kBrake);

        config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Units.radiansToRotations(absoluteOffset);
        config.MagnetSensor.SensorDirection = isCancoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder = new CANcoder(canCoderID);
        absoluteEncoder.getConfigurator().apply(config);


        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(turnEncoderToRadian);
        turnEncoder.setVelocityConversionFactor(turnEncoderRPMToRadPerSec);

        //Creating and configuring PID controllers
        turningPID = new PIDController(PTurn, ITurn, DTurn);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
        Constants.ModuleConstants.tab.add(turningPID).withWidget(BuiltInWidgets.kPIDController);

        resetEncoders();

    }

    public void playNote(double hz){
        driveMotor.setControl(new MusicTone(hz));
    }

    /**
     * Gets the drive position from the drive encoder
     * 
     * @return drivePosition -the position of the relative encoder
     */
    public double getDrivePosition() {
        //return driveMotor.getSelectedSensorPosition() * driveEncoderToMeter;
        return driveMotor.getPosition().getValueAsDouble() * (0.32 / 13824);
    }

    /**Returns position of turn encoder in radians. Counterclockwise is positive, accumulates. */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /**Returns the rotation2d of the swerve module.*/
    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(getTurnPosition());
        return Rotation2d.fromRadians(getTurnPosition());
    }

    /**Returns the velocity of the drive motor in m/s. */
    public double getDriveVelocity() {
        return driveMotor.getPosition().getValueAsDouble() * driveEncoderRPMToMeterPerSec;
    }

    /**Returns the velocity of the turn motor. */
    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
        //>return turnMotor.getSelectedSensorVelocity();
    }

    /**Returns the absolute position of the CANcoder. */
    public double getAbsoluteTurnPosition() {
        return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble()); // * 360) * (180.0/Math.PI);
    }

    /**Returns the current of the drive motor. */
    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**Returns the current of the turn motor. */
    public double getTurnCurrent() {
        return turnMotor.getOutputCurrent();
    }

    /**Resets the relative turn motor encoder to the absolute turn position of the CANcoder. */
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turnEncoder.setPosition(getAbsoluteTurnPosition());
        //System.out.println("RESETTING ENCODERS \nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS\nRESETTING ENCODERS");
    }

    /**Returns the state of the swerve module. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    /**Get the module position. */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    /**Sets the desired state of the swerve module. */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        if(state.speedMetersPerSecond > 0) {
            //driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeed);
        wantedSpeed = (((state.speedMetersPerSecond / maxSpeed) * 0.94) + 0.06);
        //driveMotor.set(ControlMode.Velocity, wantedSpeed * 3 / 2);
        // I LOVE ALIVEBAND  
        driveMotor.set(((state.speedMetersPerSecond / maxSpeed) * 0.94) + 0.06);
            
        } else {
            //driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / maxSpeed);
        wantedSpeed = (((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        //driveMotor.set(ControlMode.Velocity, wantedSpeed * 3 / 2);
        // I LOVE ALIVEBAND  
        driveMotor.set(((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.0);
        }
        
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));

    }

    /**Sets the state of the swerve module. */
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        //driveMotor.set(ControlMode.PercentOutput, -((state.speedMetersPerSecond / maxSpeed) * 0.94) - 0.06);
        driveMotor.set(-state.speedMetersPerSecond / maxSpeed);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    /**Returns the wanted speed. */
    public double getWantedSpeed() {
        return wantedSpeed;
    }

    /**Stops the swerve module. */
    public void stop() {
        wantedSpeed = 0;
        //driveMotor.set(ControlMode.Velocity, 0);
        driveMotor.set(0);
        turnMotor.set(0);
    }


    //EXPERIMENTAL METHODS:

    /**
     * Uses a PID to set drive velocity to 0
     */
    public void semiAutoStop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**Sets the velocity of the drive motor to 0 and the speed of the turn motor to 0. */
    public void fullStop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
    /**
     * Sets wheels to X formation
     */
    public void activeStop(int direction) {
        //System.out.println("2\n2\n2\n2\n2\n2\n2\n2");
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0.785398 * direction));
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(0);
        turnMotor.set(turningPID.calculate(getTurnPosition(), state.angle.getRadians()));
    }
}
