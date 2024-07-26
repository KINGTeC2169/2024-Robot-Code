package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteManager;

public class Pickup extends Command {
    private Intake intake;
    private boolean started;
    private double localMax;

    public Pickup(Intake intake){
        this.intake = intake;
        addRequirements(intake);
        localMax = 0;
    }

    @Override
    public void initialize(){
        started = false;
        localMax = 0;
    }

    @Override
    //Runs intake
    public void execute() { 
        intake.inTake();
        if(intake.getMode() == 1){
            if(intake.getRPM() > localMax) localMax = intake.getRPM();
            if (intake.getRPM() < localMax-200){
                NoteManager.setTrue();
            }
        }
    }

    @Override
    //This is a small adjustment of the note in case the beambreak is too slow. It runs the intake backwards for a few milliseconds to make sure the note doesn't touch the shooter wheels.
    public void end(boolean interupt) {
        //System.out.println("stop");
        intake.stopTake();
        intake.outTake();
        Timer.delay(0.02);
        intake.stopTake();
	}

    @Override
    //Returns true if there is a note in the intake
	public boolean isFinished() {
        //if(intake.getMode() != 0){
		return NoteManager.hasNote();
        //}
        //return false;
	}
}
