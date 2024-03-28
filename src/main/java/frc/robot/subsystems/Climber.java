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

    //No touchy touchie
    private DigitalInput leftTouchy;
    private DigitalInput rightTouchie;

    public Climber(){
        leftClimber = new CANSparkMax(Ports.leftClimber, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Ports.rightClimber, MotorType.kBrushless);

        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        leftTouchy = new DigitalInput(Ports.leftClimberTouch);
        rightTouchie = new DigitalInput(Ports.rightClimberTouch);

        tab.addDouble("Left Arm Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max" , 12)).withPosition(0, 0);
        tab.addDouble("Right Arm Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max" , 12)).withPosition(0, 1);

        tab.addDouble("Left Amperage", () -> getAmperage()[0]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max" , 30)).withPosition(2, 0).withSize(2, 2);
        tab.addDouble("Right Amperage", () -> getAmperage()[1]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max" , 30)).withPosition(2, 2).withSize(2, 2);

        tab.addBoolean("Left Touch Sensor", () -> getTouchies()[0]).withPosition(0, 4);
        tab.addBoolean("Right Touch Sensor", () -> getTouchies()[1]).withPosition(0, 5);

        leftEncoder = leftClimber.getEncoder();
        rightEncoder = rightClimber.getEncoder();
     
    }

    public double[] getPosition(){
        return new double[] {leftEncoder.getPosition(),
                             rightEncoder.getPosition()};
    }

    public double[] getVoltage(){
        return new double[] {leftClimber.getBusVoltage(),
                             rightClimber.getBusVoltage()};
    }

    public double[] getAmperage(){
        return new double[] {leftClimber.getOutputCurrent(),
                             rightClimber.getOutputCurrent()};
    }

    public boolean[] getTouchies(){
        return new boolean[]{!leftTouchy.get(),
                             !rightTouchie.get()};
    }

    public void setSpeed(double speed){
        leftClimber.set(speed);
        rightClimber.set(speed);
    }

    public void setLeftSpeed(double speed){
        leftClimber.set(speed);
    }

    public void setRightSpeed(double speed){
        rightClimber.set(speed);
    }  
    
    public void setMaxHeight(){
        this.setSpeed(0.5);
        Timer.delay(0.5);
        this.setSpeed(0);
    }

    public void setRightMaxHeight(){
        this.setRightSpeed(0.5);
        Timer.delay(0.5);
        this.setRightSpeed(0);
    }

    public void setLeftMaxHeight(){
        this.setLeftSpeed(0.5);
        Timer.delay(0.5);
        this.setLeftSpeed(0);
    }

    public boolean leftClimberDown(){
        return getTouchies()[0];
    }

    public boolean rightClimberDown(){
        return getTouchies()[1];
    }

    public boolean climberDown(){
        return getTouchies()[0] && getTouchies()[1];
    }

}
