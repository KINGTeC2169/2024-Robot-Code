// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MusicCommand;
import frc.robot.commands.ButtonCommands.ArmStop;
import frc.robot.commands.ButtonCommands.GroundPickup;
import frc.robot.commands.ButtonCommands.Launch;
import frc.robot.commands.ButtonCommands.Safe;
import frc.robot.commands.ButtonCommands.Stop;
import frc.robot.commands.ButtonCommands.ShootingPos.Amp;
import frc.robot.commands.ButtonCommands.ShootingPos.Angle;
import frc.robot.commands.ButtonCommands.ShootingPos.LimelightAlign;
import frc.robot.commands.ButtonCommands.ShootingPos.Podium;
import frc.robot.commands.ButtonCommands.ShootingPos.RevAndAngle;
import frc.robot.commands.ButtonCommands.ShootingPos.Subwoofer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
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

  private final Pigeon pigeon = new Pigeon();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //Driver Station:
  private final Joystick leftStick = new Joystick(Constants.Ports.leftStick);
  private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.controller);
  private final CommandXboxController buttonBoard = new CommandXboxController(Constants.Ports.buttons);

  private ShuffleboardTab tab = Shuffleboard.getTab("Shuffleboard");

  private GenericEntry songChoice = tab.add("Song Choice", 0.0).getEntry();
  
  private Command themeSong; 
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Register Named Commands
    NamedCommands.registerCommand("Ground Pickup", new GroundPickup(arm, intake));
    NamedCommands.registerCommand("Amp Launch", new Amp(arm, shooter));
    NamedCommands.registerCommand("Podium Launch", new Podium(arm, shooter));
    NamedCommands.registerCommand("Rev and Launch 1", new RevAndAngle(arm, shooter, 0)); 
    NamedCommands.registerCommand("Rev and Launch 2", new RevAndAngle(arm, shooter, 0));
    
    //Initalize autos
    //PathPlannerAuto ampPath = PathPlannerAuto.getPathGroupFromAutoFile("2 Ring Amp");

    // Configure the trigger bindings

    themeSong = new MusicCommand(swerveSubsystem, "src\\main\\deploy\\ThunderStruck.chrp");

    HashMap<String, Command> ampMap = new HashMap<String, Command>();

    swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem,
    () -> leftStick.getY(),
    () -> leftStick.getX(),
    () -> rightStick.getTwist(),
		() -> rightStick.getRawButton(1),
    () -> !leftStick.getRawButton(1)
		));

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

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
    controller.leftBumper().whileTrue(Commands.run(() -> arm.armStop())); 
    controller.rightBumper().whileTrue(new Angle(arm, controller));
    controller.b().whileTrue(Commands.run(() -> shooter.setRPM(4400)));
    controller.a().whileTrue(Commands.run(() -> shooter.stopShooter()));
    controller.x().whileTrue(Commands.run(() -> intake.outTake()));
    controller.y().whileTrue(Commands.run(() -> intake.inTake()));



    buttonBoard.button(1).whileTrue(new GroundPickup(arm, intake));
    buttonBoard.button(2).whileTrue(new Safe(arm, intake));
    buttonBoard.button(3).whileTrue(new Subwoofer(arm, shooter));
    buttonBoard.button(4).whileTrue(new Amp(arm, shooter));
    buttonBoard.button(5).whileTrue(Commands.run(() -> shooter.setRPM(4400)));
    buttonBoard.button(6).whileTrue(new Launch(intake));
    buttonBoard.button(7).whileTrue(new LimelightAlign(swerveSubsystem, arm, shooter));
    buttonBoard.button(8).whileTrue(new Podium(arm, shooter));
    buttonBoard.button(9).whileTrue(new Stop(shooter, intake));
    buttonBoard.button(10).onTrue(Commands.run(() -> arm.armStop()));
    buttonBoard.button(11).whileTrue(Commands.run(() -> intake.outTake()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public Command getTestCommand(){
      return null;
  }
}
