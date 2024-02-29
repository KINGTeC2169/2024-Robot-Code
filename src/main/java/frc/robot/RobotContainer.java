// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Positions;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ButtonCommands.GroundPickup;
import frc.robot.commands.ButtonCommands.IntakeCommand;
import frc.robot.commands.ButtonCommands.Pickup;
import frc.robot.commands.ButtonCommands.Rest;
import frc.robot.commands.ButtonCommands.RevAndAngle;
import frc.robot.commands.ButtonCommands.Launch;
import frc.robot.commands.ButtonCommands.LimelightAlign;
import frc.robot.commands.ButtonCommands.Outtake;
import frc.robot.commands.ButtonCommands.Shoot;
import frc.robot.commands.ButtonCommands.Stop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightTable;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LimelightTable limelight = new LimelightTable();
  private final Pigeon pigeon = new Pigeon();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //Driver Station controllers
  private final Joystick leftStick = new Joystick(Constants.Ports.leftStick);
  private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.controller);
  private final CommandXboxController buttonBoard = new CommandXboxController(Constants.Ports.buttons);

  //Shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("Auto Chooser");
  private GenericEntry autoChoice = tab.add("Auto Choice", 0).withPosition(3, 0).getEntry();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Register Named Commands
    NamedCommands.registerCommand("Ground Pickup", new GroundPickup(arm, intake));
    NamedCommands.registerCommand("Amp Launch", new RevAndAngle(arm, shooter, Positions.amp));
    NamedCommands.registerCommand("Podium Launch", new RevAndAngle(arm, shooter, Positions.podium));
    NamedCommands.registerCommand("Rev and Launch 1", new RevAndAngle(arm, shooter, 0)); 
    NamedCommands.registerCommand("Rev and Launch 2", new RevAndAngle(arm, shooter, 0));
    NamedCommands.registerCommand("Rev and Launch 3", new RevAndAngle(arm, shooter, 0));

    //themeSong = new MusicCommand(swerveSubsystem, "src\\main\\deploy\\ThunderStruck.chrp");
    tab.addString("Auto 1.0", () -> "3 Ring Center").withSize(3, 1).withPosition(0, 0);
    tab.addString("Auto 2.0", () -> "2 Ring Amp").withSize(3, 1).withPosition(0, 1);
    tab.addString("Auto 3.0", () -> "2 Ring Source").withSize(3, 1).withPosition(0, 2);
    tab.addString("Auto 0.0", () -> "Just Drive").withSize(3, 1).withPosition(0, 3);

    swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem,
    () -> leftStick.getY(),
    () -> leftStick.getX(),
    () -> rightStick.getTwist(),
		() -> rightStick.getRawButton(2),
    () -> rightStick.getRawButton(1),
    () -> leftStick.getRawButton(2),
    () -> leftStick.getRawButton(1)
		));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Jalol controls:
    /* 
    controller.povDown().whileTrue(Commands.run(() -> intake.outTake()));
    controller.povUp().whileTrue(new Safe(arm, intake));
    controller.back().whileTrue(new Stop(shooter, intake));
    controller.start().whileTrue(new ArmStop(arm));
    controller.leftBumper().whileTrue(new GroundPickup(arm, intake));
    controller.rightBumper().whileTrue(Commands.run(() -> shooter.setRPM(4400)));
    controller.leftTrigger(0.2).whileTrue(new Safe(arm, intake));
    controller.rightTrigger(0.2).whileTrue(new Launch(intake));
    controller.y().whileTrue(new Subwoofer(arm, shooter));
    controller.x().whileTrue(new LimelightAlign(swerveSubsystem, arm, shooter));
    controller.a().whileTrue(new Podium(arm, shooter));
    controller.b().whileTrue(new Amp(arm, shooter));
    */

    //Testing controls:
    controller.leftBumper().whileTrue(Commands.run(() -> arm.setSpeed(-controller.getRightY()/10.0)));
    controller.rightBumper().whileTrue(new Shoot(shooter));

    //Comp controls:
    controller.leftTrigger(0.2).whileTrue(new Rest(arm));
    controller.rightTrigger(0.2).whileTrue(new IntakeCommand(intake));

    controller.back().whileTrue(new Rest(arm));
    controller.start().whileTrue(new Rest(arm));

    controller.povDown().whileTrue(new Outtake(intake));
    controller.povUp().whileTrue(new IntakeCommand(intake));

    controller.y().whileTrue(new RevAndAngle(arm, shooter, Positions.subwoofer));
    controller.x().whileTrue(new RevAndAngle(arm, shooter, Positions.sideSubwoofer));
    controller.a().whileTrue(new RevAndAngle(arm, shooter, Positions.podium));
    controller.b().whileTrue(new RevAndAngle(arm, shooter, Positions.amp));

    controller.povRight().whileTrue(new Pickup(intake));
    controller.povUp().whileTrue(new Outtake(intake));
    controller.back().whileTrue(new Launch(intake));
    controller.povLeft().whileTrue(new LimelightAlign(swerveSubsystem, arm, shooter));



    buttonBoard.button(1).whileTrue(new GroundPickup(arm, intake));
    buttonBoard.button(2).whileTrue(new Rest(arm));
    buttonBoard.button(3).whileTrue(new RevAndAngle(arm, shooter, Positions.subwoofer));
    buttonBoard.button(4).whileTrue(new RevAndAngle(arm, shooter, Positions.amp));
    buttonBoard.button(5).whileTrue(new RevAndAngle(arm, shooter, Positions.sideSubwoofer));
    buttonBoard.button(6).whileTrue(new Launch(intake));
    buttonBoard.button(7).whileTrue(new LimelightAlign(swerveSubsystem, arm, shooter));
    buttonBoard.button(8).whileTrue(new RevAndAngle(arm, shooter, Positions.podium));
    buttonBoard.button(9).whileTrue(new Stop(shooter, intake)); 
    buttonBoard.button(10).onTrue(Commands.run(() -> arm.activeStop()));
    buttonBoard.button(11).whileTrue(Commands.run(() -> intake.outTake()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChoice.getDouble(0.0) == 1.0){
      return new PathPlannerAuto("3 Ring Center");
    }

    else if (autoChoice.getDouble(0.0) == 2.0){
      return new PathPlannerAuto("2 Ring Amp");
    }

    else if (autoChoice.getDouble(0.0) == 3.0){
      return new PathPlannerAuto("2 Ring Source");
    }

    else if (autoChoice.getDouble(0.0) == 4.0){
      
    }
    return new PathPlannerAuto("Just Drive");
  }
}
