package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Climber extends SubsystemBase{
    
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;
    private ShuffleboardTab tab = Shuffleboard.getTab("Climber");

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private DigitalInput leftLimit;
    private DigitalInput rightLimit;

    public Climber(){
        leftClimber = new CANSparkMax(Ports.leftClimber, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Ports.rightClimber, MotorType.kBrushless);

        //Sets the neos to break mode so they can hold the robot
        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        //Defines the limit switches
        leftLimit = new DigitalInput(Ports.leftClimberTouch);
        rightLimit = new DigitalInput(Ports.rightClimberTouch);

        tab.addDouble("Left Arm Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max" , 12)).withPosition(0, 0);
        tab.addDouble("Right Arm Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max" , 12)).withPosition(0, 1);

        tab.addDouble("Left Amperage", () -> getAmperage()[0]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max" , 30)).withPosition(2, 0).withSize(2, 2);
        tab.addDouble("Right Amperage", () -> getAmperage()[1]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max" , 30)).withPosition(2, 2).withSize(2, 2);

        tab.addBoolean("Left Touch Sensor", () -> getLimits()[0]).withPosition(0, 3);
        tab.addBoolean("Right Touch Sensor", () -> getLimits()[1]).withPosition(1, 3);

        leftEncoder = leftClimber.getEncoder();
        rightEncoder = rightClimber.getEncoder();
     
    }

    /**Returns the encoder postions for the left and right motor. */
    public double[] getPosition(){
        return new double[] {leftEncoder.getPosition(),
                             rightEncoder.getPosition()};
    }

    /**Returns the voltage input for the left and right motors. */
    public double[] getVoltage(){
        return new double[] {leftClimber.getBusVoltage(),
                             rightClimber.getBusVoltage()};
    }

    /**Returns the current output of the left and right motors. */
    public double[] getAmperage(){
        return new double[] {leftClimber.getOutputCurrent(),
                             rightClimber.getOutputCurrent()};
    }

    /**Returns a boolean array with the limit switch values of the climbers */
    public boolean[] getLimits(){
        return new boolean[]{!leftLimit.get(),
                             !rightLimit.get()};
    }

    /**Sets the speed of both the climber motors. 
     * @param speed The speed to set.
    */
    public void setSpeed(double speed){
        leftClimber.set(speed);
        rightClimber.set(speed);
    }

    /**Sets the left climber speed. 
     * @param speed The speed to set. Must be above 0.1.
     */
    public void setLeftSpeed(double speed){
        // if(Math.abs(speed)<0.1 && !getLimits()[0]){
        //     leftClimber.set(0);
        // } else {
            if(Math.abs(speed)<0.08){
                leftClimber.set(0);
            }else{
                leftClimber.set(speed);
            }
        // }
    }

    /**Sets the right climber speed. 
     * @param speed The speed to set. Must be above 0.1.
     */
    public void setRightSpeed(double speed){
        // if(Math.abs(speed)<0.1 && !getLimits()[0]){
        //     rightClimber.set(0);
        // } else {
            if(Math.abs(speed)<0.08){
                rightClimber.set(0);
            }else{
                rightClimber.set(speed);
            }
        // }
    }  
    
    /**Runs the climber motors for 0.5 seconds to set it to its max height. */
    public void setMaxHeight(){
        this.setSpeed(0.5);
        Timer.delay(0.5);
        this.setSpeed(0);
    }

    /**Runs the right climber motor for 0.5 seconds to set it to its max height. */
    public void setRightMaxHeight(){
        this.setRightSpeed(0.5);
        Timer.delay(0.5);
        this.setRightSpeed(0);
    }

    /**Runs the left climber motor for 0.5 seconds to set it to its max height. */
    public void setLeftMaxHeight(){
        this.setLeftSpeed(0.5);
        Timer.delay(0.5);
        this.setLeftSpeed(0);
    }

    /**Returns true if the left climber limit switch is triggered. */
    public boolean leftClimberDown(){
        return getLimits()[0];
    }

    /**Returns true if the right climber limit switch is triggered. */
    public boolean rightClimberDown(){
        return getLimits()[1];
    }

    /**Returns true if both climber limit switches are triggered. */
    public boolean climberDown(){
        return getLimits()[0] && getLimits()[1];
    }

}
