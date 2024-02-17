package frc.robot.commands.ButtonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    private Intake intake;
    private CommandXboxController controller;

    public IntakeNote(Intake intake, CommandXboxController controller){
        this.intake = intake;
        addRequirements(intake);
        this.controller = controller;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() { 
        intake.inTake();
    }

    @Override
    public void end(boolean interupt) {
        intake.stopTake();
	}

    @Override
	public boolean isFinished() {
		return true;
	}
}
