package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    public Climber(){
        leftClimber = new CANSparkMax(Ports.leftClimber, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Ports.rightClimber, MotorType.kBrushless);

        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        tab.addDouble("Left Arm Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max" , 12)).withPosition(0, 0);
        tab.addDouble("Right Arm Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max" , 12)).withPosition(0, 1);

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

    public boolean climberDown(){
        return false;
    }

}
