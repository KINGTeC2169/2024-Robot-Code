// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.Positions;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LetsFlyLeft;
import frc.robot.commands.LetsFlyRight;
import frc.robot.commands.ShootArm.Stuck;
import frc.robot.commands.ShootArm.VisionAim;
import frc.robot.commands.Intake.Launch;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.Intake.Pickup;
import frc.robot.commands.ShootArm.Amp;

import frc.robot.commands.ShootArm.Rest;
import frc.robot.commands.ShootArm.RevAndAngle;
import frc.robot.commands.ShootArm.RevAngleLaunch;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightTable;
import frc.robot.subsystems.Music;
import frc.robot.subsystems.NoteManager;
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
  @SuppressWarnings("unused")
  private final LimelightTable limelight = new LimelightTable();
  @SuppressWarnings("unused")
  private final Pigeon pigeon = new Pigeon();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake(); //also change in note manager
  private final Shooter shooter = new Shooter();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Music music = new Music(arm,shooter,intake,swerveSubsystem);

  //Driver Station controllers
  private final Joystick leftStick = new Joystick(Constants.Ports.leftStick);
  private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.controller);
  //private final XboxController controllerRumble = new XboxController(Constants.Ports.controller);
  private final CommandXboxController buttonBoard = new CommandXboxController(Constants.Ports.buttons);

  //Shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("Auto Chooser");
  private GenericEntry autoChoice = tab.add("Auto Choice", 0).withPosition(6, 0).getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Register Named Commands
    NamedCommands.registerCommand("Ground Pickup", new Pickup(intake));
    NamedCommands.registerCommand("Amp Launch", new RevAngleLaunch(arm, shooter, intake, Positions.amp));
    NamedCommands.registerCommand("Rest Position", new Rest(arm));
    NamedCommands.registerCommand("Subwoofer Launch", new RevAngleLaunch(arm, shooter, intake, Positions.subwoofer));
    NamedCommands.registerCommand("Podium Launch", new RevAngleLaunch(arm, shooter, intake, Positions.podium));
    NamedCommands.registerCommand("Sidesub Launch", new RevAngleLaunch(arm, shooter, intake, Positions.sideSubwoofer));
    NamedCommands.registerCommand("Half Court Launch", new RevAngleLaunch(arm, shooter, intake, 0.34, 1000));

    //themeSong = new MusicCommand(swerveSubsystem, "src\\main\\deploy\\ThunderStruck.chrp");
    tab.addString("Auto 1.0", () -> "3 Ring Center").withSize(3, 1).withPosition(0, 0);
    tab.addString("Auto 2.0", () -> "2 Ring Amp").withSize(3, 1).withPosition(0, 1);
    tab.addString("Auto 3.0", () -> "Just Shoot").withSize(3, 1).withPosition(0, 2);
    tab.addString("Auto 4.0", () -> "Center Line Amp").withSize(3, 1).withPosition(0, 3);
    tab.addString("Auto 5.0", () -> "Sanju one").withSize(3, 1).withPosition(3, 0);
    tab.addString("Auto 6.0", () -> "Hogging Source").withSize(3, 1).withPosition(3, 1);
    tab.addString("Auto 0.0", () -> "Just Drive").withSize(3, 1).withPosition(3, 2);

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

    //Comp controls:

    //controller.getAButton().onTrue(new RevAndAngle(arm, shooter, Positions.subwoofer));
    controller.leftBumper().whileTrue(Commands.run(() -> arm.setVoltage(-controller.getRightY()*2)));
    controller.rightBumper().onTrue(Commands.run(() -> shooter.setRPM(3000)));
    controller.rightBumper().onFalse(Commands.run(() -> shooter.setRPM(0)));

    controller.leftTrigger(0.2).onTrue(new Pickup(intake));
    //controller.leftTrigger(0.2).whileFalse(Commands.run(() -> intake.stopTake()));
    controller.rightTrigger(0.2).whileTrue(new Launch(intake));
    controller.rightTrigger(0.2).onFalse(new Rest(arm));

    controller.povUp().onTrue(new RevAndAngle(arm, shooter, 0.34));
    controller.povDown().onTrue(new Rest(arm));
    controller.x().whileTrue(new Outtake(intake));
    controller.povRight().onTrue(Commands.run(() -> NoteManager.setTrue()));
    controller.povRight().whileFalse(Commands.run(() -> NoteManager.setFalse()));
    //controller.a().onTrue(new RevAndAngle(arm, shooter, Positions.subwoofer));
    controller.a().onTrue(new RevAndAngle(arm, shooter, Positions.subwoofer));
    /*controller.a().onTrue(Commands.run(() -> controllerRumble.setRumble(GenericHID.RumbleType.kRightRumble, 0.2)));
    controller.a().onTrue(Commands.run(() -> controllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0.2)));
    controller.b().onTrue(Commands.run(() -> controllerRumble.setRumble(GenericHID.RumbleType.kRightRumble, 0)));
    controller.b().onTrue(Commands.run(() -> controllerRumble.setRumble(GenericHID.RumbleType.kLeftRumble, 0)));
    */
    //controller.x().onTrue(new RevAndAngle(arm, shooter, Arm.aimToArm(LimelightTable.aimShot())));
    controller.y().onTrue(new RevAndAngle(arm, shooter, Positions.sideSubwoofer));
    //0.37
    //controller.y().onTrue(new RevAndAngle(arm, shooter, Positions.podium));;
    controller.b().onTrue(new Amp(arm, shooter));

    controller.back().whileTrue(new Stuck(arm, shooter));
    controller.start().onTrue(Commands.run(() -> intake.cycleMode()));

    // controller.leftBumper().whileFalse(new LetsFlyLeft(climber, -controller.getLeftY()));
    // controller.leftBumper().whileFalse(new LetsFlyRight(climber, -controller.getRightY()));
    controller.leftBumper().whileFalse(Commands.run(() -> climber.setLeftSpeed(-controller.getLeftY())));
    controller.leftBumper().whileFalse(Commands.run(() -> climber.setRightSpeed(-controller.getRightY())));

    //controller.y().whileTrue(Commands.run(() -> arm.updatePIDF()))
    //controller.povUp().whileTrue(new IntakeCommand(intake, arm));
    //controller.a().whileTrue(Commands.run(() -> arm.posOne()));
    //controller.b().whileTrue(Commands.run(() -> arm.posZero()));
    //controller.x().whileTrue(Commands.run(() -> arm.updatePID()));
    //controller.x().onTrue(new RevAndAngle(arm, shooter, Positions.sideSubwoofer));
    //controller.y().whileTrue(Commands.run(() -> music.play()));
    //controller.povUp().whileTrue(new Outtake(intake));
    //controller.back().whileTrue(new Launch(intake));
    //controller.povLeft().onTrue(new RevAngleLaunch(arm, shooter, intake, Positions.subwoofer));


    buttonBoard.button(1).whileTrue(new Pickup(intake));
    buttonBoard.button(2).whileTrue(new Outtake(intake));
    buttonBoard.button(3).whileTrue(new RevAndAngle(arm, shooter, Positions.subwoofer));
    buttonBoard.button(4).whileTrue(new RevAndAngle(arm, shooter, Positions.sideSubwoofer));
    buttonBoard.button(5).onTrue(new Pickup(intake));
    buttonBoard.button(6).onTrue(new Rest(arm));
    buttonBoard.button(7).whileTrue(new Amp(arm, shooter));
    buttonBoard.button(8).whileTrue(new RevAndAngle(arm, shooter, Positions.podium));
    buttonBoard.button(10).whileTrue((Commands.run(() -> music.play())));
    //buttonBoard.button(10).onTrue(Commands.run(() -> arm.activeStop()));
    //buttonBoard.button(11).whileTrue(new Angle(arm, 0.49));
    
  }


  public Command getTestCommand(){
    return Commands.run(music::play);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    swerveSubsystem.zeroHeading();

    if (autoChoice.getDouble(0.0) == 1.0){
      return new PathPlannerAuto("3 Ring Center");
    }

    else if (autoChoice.getDouble(0.0) == 2.0){
      return new PathPlannerAuto("2 Ring Amp");
    }

    else if (autoChoice.getDouble(0.0) == 3.0){
      return new PathPlannerAuto("Shoot");
    }

    else if (autoChoice.getDouble(0.0) == 4.0){
      return new PathPlannerAuto("Center Line Amp");
    }

    else if (autoChoice.getDouble(0.0) == 5.0){
      return new PathPlannerAuto("Sanju one");
    }
  
    else if (autoChoice.getDouble(0.0) == 6.0){
      return new PathPlannerAuto("Hogging Source");
    }
    return new PathPlannerAuto("Just Drive");
  }
}
