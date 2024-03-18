package frc.robot.subsystems;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class Music {
    
    private Orchestra orchestra;
    private String songPath;
    private TalonFX motor;

    public Music(){
        orchestra = new Orchestra();
        motor = new TalonFX(30);
        //orchestra.addInstrument(motor);
        orchestra.loadMusic("Thunderstruck.chrp");
        
    }

    public void play(){
        motor.setControl(new MusicTone(200));
        Timer.delay(0.2);
        motor.setControl(new MusicTone(400));
        Timer.delay(0.2);
        motor.setControl(new MusicTone(800));
        Timer.delay(0.2);
        motor.setControl(new MusicTone(400));
        Timer.delay(0.2);
        motor.setControl(new MusicTone(200));
        Timer.delay(0.2);
        motor.setControl(new MusicTone(400));
        Timer.delay(0.2);
        //orchestra.play();
    }
}
