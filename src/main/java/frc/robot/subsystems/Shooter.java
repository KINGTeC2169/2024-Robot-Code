package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;

public class Shooter extends SubsystemBase {
    private final VelocityVoltage m_velocity = new VelocityVoltage(0);
    
    private TalonFX shooterTop;
    private TalonFX shooterBot;

    private double topRotation = 1; //24t on flywheel/24t on 
    private double botRotation = 1.333; //24t on flywheel/18t on motor shaft

    private double setpointRPM = 0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    //private GenericEntry shooterSpeed = tab.add("Shooter Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("Min", 0)).withPosition(4, 0).getEntry();

    public Shooter(){
        
        shooterTop = new TalonFX(Constants.Ports.shooterTop);
        shooterBot = new TalonFX(Constants.Ports.shooterBot);

        //Shooter motor configuration
        var configs = new Slot0Configs();
        configs.kP = 0.1;
        
        shooterTop.getConfigurator().apply(configs,0.050);
        shooterBot.getConfigurator().apply(configs,0.050);
        shooterTop.setInverted(true);
        shooterBot.setInverted(true);

        ShuffleboardLayout topMotor = tab.getLayout("Top Motor", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
        topMotor.addDouble("Top Motor RPM", () -> getRPM()[0]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 4000));
        topMotor.addDouble("Top Motor Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max", 12));
        topMotor.addDouble("Top Motor Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial);

        ShuffleboardLayout bottomMotor = tab.getLayout("Bottom Motor", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
        bottomMotor.addDouble("Bottom Motor RPM", () -> getRPM()[1]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 4000));
        bottomMotor.addDouble("Bottom Motor Voltage", () -> getVoltage()[1]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max", 12));
        bottomMotor.addDouble("Bottom Motor Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial);

        tab.addBoolean("Shooter Ready", () -> shooterReady()).withPosition(6, 0);

    }

    /**Sets the speed of the shooter
     * @param power double between 1 and -1
     */ 
    public void setPower(double power){

        shooterTop.set(-power);
        shooterBot.set(-power);
        SmartDashboard.putNumber("Request ShotSpeed", power);
    }

    /**Gets the current from the top and bottom motors as a double array */
    public double[] getCurrent(){
        return new double[]{shooterTop.getSupplyCurrent().getValueAsDouble(),
                            shooterBot.getSupplyCurrent().getValueAsDouble()};
    }

    /**Gets the voltage from the top and bottom motors as a double array */
    public double[] getVoltage() {    
        return new double[]{shooterTop.getSupplyVoltage().getValueAsDouble(),
                            shooterBot.getSupplyVoltage().getValueAsDouble()};
    }
    
    /**Gets the rpm from the top and bottom motors as a double array*/
    public double[] getRPM(){
        return new double[]{(-(60 * shooterTop.getRotorVelocity().getValue())),
                            (-(60 * shooterBot.getRotorVelocity().getValue()))};
    }

    /**Gets the shooter rpm from the top and bottom motors as a double array*/
    public double[] getShooterRPM(){
        return new double[]{(-(60 * shooterTop.getRotorVelocity().getValue()) * topRotation),
                            (-(60 * shooterBot.getRotorVelocity().getValue()) * botRotation)};
    }

    public void setRPM(double rpm){
        m_velocity.Slot = 0;
        setpointRPM = rpm;
        double topRPS = topRotation * rpm/60 * 2.14285714285714;
        double botRPS = botRotation * rpm/60 * 2.14285714285714;
        System.out.println(topRPS);
        System.out.println(botRPS + "b");
        shooterTop.setControl(m_velocity.withVelocity(topRPS));
        shooterBot.setControl(m_velocity.withVelocity(botRPS));
    }

    /**Sets the shooter to the rpm for shooting*/
    public void shootRPM(){
        setRPM(Vision.shootRPM);
    }

    /**Runs the shooter at the rpm for scoring amp*/
    public void ampRPM(){
        setRPM(Vision.ampRPM);
    }

    /**Runs the shooter backward */
    public void backward(){
        setRPM(-1000);
    }

    /**Returns true if both shooter motors' rpm is +/- 200 rpm of the desired rpm */
    public boolean shooterReady(){
        return Math.abs(getShooterRPM()[0] - setpointRPM) < 200 &&  Math.abs(getShooterRPM()[1] - setpointRPM) < 200;
    }

    /**Sets the shooter speed to 0 */
    public void stopShooter() {
        shooterTop.set(0); 
        shooterBot.set(0);
    }

    /**Returns true if the current of either shooter motors is greater than 0 */
    public boolean on(){
        return getCurrent()[0] > 0 || getCurrent()[1] > 0;
    }

    /**Used for music */
     public void playNote(double hz){
        shooterTop.setControl(new MusicTone(hz));
        shooterBot.setControl(new MusicTone(hz));
    }
}