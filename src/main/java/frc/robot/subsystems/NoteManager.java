package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteManager extends SubsystemBase {

    //private DigitalInput beamBreak = new DigitalInput(Constants.Ports.beamBreak);

    /**Returns true if there is something in the beambreak */
    public boolean isBall() {
        //return !beamBreak.get();
        return true;
    }
    
}
