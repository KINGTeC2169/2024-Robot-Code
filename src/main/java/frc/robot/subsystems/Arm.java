package frc.robot.subsystems;


import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

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

    //private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.Ports.armEncoder);
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

        ShuffleboardLayout currents = tab.getLayout("Arm Currents", BuiltInLayouts.kGrid).withSize(2, 1).withProperties(Map.of("Number of rows", 1)).withPosition(0, 0);

        var configs = new Slot0Configs();
        configs.kP = 0.5;
        
        leftArm.getConfigurator().apply(configs,0.050);
        rightArm.getConfigurator().apply(configs,0.050);
        
        setAngle = 0;

        zero = getOGAngle();

        tab.addDouble("Absolute Angle", () -> getAngle());
        tab.addDouble("Zero Angle", () -> zero);
        tab.addBoolean("Arm Ready", () -> armReady());
        currents.addDouble("Left Arm Volt", () -> getArmVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView);
        currents.addDouble("Right Arm Volt", () -> getArmVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView);

    }

    public double getEncoderAngle() {
        //return (armEncoder.getAbsolutePosition() - armEncoder.getPositionOffset()); //todo: test how much is 1 rotation
        return 0;
    }

    public double getOGAngle(){
        return leftArm.getPosition().getValueAsDouble();
    }

    public double getAngle() {
        return leftArm.getPosition().getValueAsDouble()-zero;
    }

    //Rotations
    public void setAngle(double angle) {
        setAngle = angle;

        leftArm.setControl(request.withPosition(angleLoop.calculate(getAngle(), angle)/12));
        rightArm.setControl(request.withPosition(angleLoop.calculate(getAngle(), angle)/12));
        
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
        return setAngle == getAngle();
    }

    public void setSpeedNoPID(double speed){
        if((this.getAngle() > armLowerLimit && speed < 0) || (speed > 0 && this.getAngle() < armUpperLimit)){
            leftArm.set(speed);
            rightArm.set(speed);
        } else { 
            this.armStop();
        }
        
    }

    public void armStop(){
        leftArm.set(0);
        rightArm.set(0);
    }

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
