// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ButtonCommands.Amp;
import frc.robot.commands.ButtonCommands.Angle;
import frc.robot.commands.ButtonCommands.GroundPickup;
import frc.robot.commands.ButtonCommands.Launch;
import frc.robot.commands.ButtonCommands.Podium;
import frc.robot.commands.ButtonCommands.RevAndAngle;
import frc.robot.commands.ButtonCommands.Subwoofer;
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

  //Right side/Buttons and Controller
  private final CommandXboxController controller = new CommandXboxController(Constants.Ports.controller);
  private final Joystick leftStick = new Joystick(Constants.Ports.leftStick);
  private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final CommandXboxController buttonBoard = new CommandXboxController(1);
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem,
		() -> leftStick.getY(), 
		() -> leftStick.getX(), 
		() -> rightStick.getTwist(),
		() -> rightStick.getRawButtonReleased(0)
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
    
    //Controller for testing until control panel is done
    controller.rightBumper().whileTrue(Commands.run(() -> new Angle(arm, true))); //Move arm up
    controller.leftBumper().whileTrue(Commands.run(() -> new Angle(arm, false))); //Move arm down
    controller.a().whileTrue(Commands.run(() -> new Launch(intake))); //Launch
    controller.b().whileTrue(Commands.run(() -> new Subwoofer(arm, shooter))); //Subwoofer
    controller.y().whileTrue(Commands.run(() -> new Amp(arm, shooter))); //Amp
    controller.start().whileTrue(Commands.run(() -> new Podium(arm, shooter))); //Podium
    controller.x().whileTrue(Commands.run(() -> new GroundPickup(arm, intake))); //Ground pickup
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
}
