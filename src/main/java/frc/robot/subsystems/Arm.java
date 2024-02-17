package frc.robot.subsystems;


import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    TalonFX leftArm = new TalonFX(Constants.DeviceID.leftArm);
    TalonFX rightArm = new TalonFX(Constants.DeviceID.rightArm);

    private final DutyCycleEncoder rightArmEncoder = new DutyCycleEncoder(Constants.Ports.rightArmEncoder);
    private final DutyCycleEncoder leftArmEncoder = new DutyCycleEncoder(Constants.Ports.leftArmEncoder);
    private final double leftZero;
    private final double rightZero;
    //Update hex encoder

    final PositionDutyCycle request = new PositionDutyCycle(0);
    PIDController angleLoop = new PIDController(0.5,0,0);

    double setAngle;
    double zero;

    //Degrees to rotations
    final double oneRotation = Constants.Motors.armGearBox;

    final double armLowerLimit = 0;
    final double armUpperLimit = (100/360); //Whatever is need for amp

    private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    
    public Arm() {

        var configs = new Slot0Configs();
        configs.kP = 0.5;
        
        leftArm.getConfigurator().apply(configs,0.050);
        rightArm.getConfigurator().apply(configs,0.050);
        
        setAngle = 0;
        leftZero = getEncoderAngle()[0];
        rightZero = getEncoderAngle()[1];

        tab.addDouble("Angle Left 0", () -> leftZero);
        tab.addDouble("Angle Right 0", () -> rightZero);
        tab.addDouble("Angle Left", () -> getEncoderAngle()[0]);
        tab.addDouble("Angle Right", () -> getEncoderAngle()[1]);
        tab.addDouble("Angle Left Off", () -> getOffAngle()[0]);
        tab.addDouble("Angle Right Off", () -> getOffAngle()[1]);
        

        tab.addBoolean("Arm Ready", () -> armReady());
        tab.addBoolean("Arm On", () -> !off());
        
        tab.addDouble("Voltage Left", () -> getArmVoltage()[0]);
        tab.addDouble("Voltage Right", () -> getArmVoltage()[1]);
        tab.addDouble("Current Left", () -> getArmCurrent()[0]);
        tab.addDouble("Current Right", () -> getArmCurrent()[1]);



    }

    public void setSpeed(double speed){
        leftArm.set(speed * 0.1);
        rightArm.set(-speed * 0.1);
    }

    public double[] getOffAngle() {
        return new double[]{(leftArmEncoder.getPositionOffset()),
                            (rightArmEncoder.getPositionOffset())}; //todo: test how much is 1 rotation
    }

    public double[] getEncoderAngle() {
        return new double[]{(leftArmEncoder.getAbsolutePosition() - leftArmEncoder.getPositionOffset()),
                            (rightArmEncoder.getAbsolutePosition() - rightArmEncoder.getPositionOffset())}; //todo: test how much is 1 rotation
    }
    /* 
    public double getAngle() {
        return leftArm.getPosition().getValueAsDouble()-zero;
    } 
    */

    //Rotations
    public void setAngle(double angle) {
        setAngle = angle;

        leftArm.setControl(request.withPosition(angleLoop.calculate(getEncoderAngle()[0], angle)/12));
        rightArm.setControl(request.withPosition(angleLoop.calculate(getEncoderAngle()[1], angle)/12));
        
        //Backup plan:
        /*
        if(getAngle() > setAngle){
            leftArm.set(-0.1);
            rightArm.set(-0.1);
        } else if(getAngle() < setAngle){
            leftArm.set(0.1);
            rightArm.set(0.1);
        } else {
            leftArm.set(0);
            rightArm.set(0);
        }
        */
    }

    public boolean armReady(){
        return setAngle == getEncoderAngle()[0] && setAngle == getEncoderAngle()[1];
    }

    public void setAngleNoPID(double angle, double speed){

        setAngle = angle;

        //Individual control:
        /* 
        if((angle >= armLowerLimit && getEncoderAngle()[0] > angle)){
            leftArm.set(0.003);
        } else if(angle <= armUpperLimit && getEncoderAngle()[0] < angle){
            leftArm.set(-0.003);
        } else {
            leftArm.set(0);
        }

        if((angle >= armLowerLimit && getEncoderAngle()[0] > angle)){
            rightArm.set(0.003);
        } else if(angle <= armUpperLimit && getEncoderAngle()[0] < angle){
            rightArm.set(-0.003);
        } else {
            rightArm.set(0);
        }*/

        if((angle >= armLowerLimit && getEncoderAngle()[0] > angle)){
            leftArm.set(speed/10);
            rightArm.set(-speed/10);
        } else if(angle <= armUpperLimit && getEncoderAngle()[0] < angle){
            leftArm.set(-speed/10);
            rightArm.set(speed/10);
        } else {
            leftArm.set(0);
            rightArm.set(0);
        }
        
    }

    public void armStop(){
        double stopLeftPos = leftArmEncoder.get();
        double stopRightPos = rightArmEncoder.get();
        leftArm.set(angleLoop.calculate(stopLeftPos, leftArm.getPosition().getValue()));
        rightArm.set(angleLoop.calculate(stopRightPos, rightArm.getPosition().getValue()));
        // leftArm.set(0);
        // rightArm.set(0);
    }

    //Extra:
    public boolean off(){

        return getArmVoltage()[0] < 5 && getArmVoltage()[1] < 5; //Todo: find rest voltage
    }

    public double[] getArmCurrent(){
        return new double[]{leftArm.getSupplyCurrent().getValueAsDouble(),
                            rightArm.getSupplyCurrent().getValueAsDouble()}; 
    }

    public double[] getArmVoltage(){
        return new double[]{leftArm.getSupplyVoltage().getValueAsDouble(),
                            rightArm.getSupplyVoltage().getValueAsDouble()}; 
    }

    
}
