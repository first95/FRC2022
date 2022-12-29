// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OI;
import frc.robot.commands.autoCommands.autoMove;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveBase drivebase = new SwerveBase();

  XboxController driverController = new XboxController(OI.DRIVER_CONTROLLER_PORT);
  //Joystick driverController = new Joystick(OI.DRIVER_CONTROLLER_PORT);
  //Joystick rotationController = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    drivebase.setDefaultCommand(
      new AbsoluteDrive(
        drivebase,
        // Applies deadbands and inverts controls because joysticks are back-right positive while robot
        // controls are front-left positive
        () -> (Math.abs(driverController.getLeftY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getLeftY() : 0,
        () -> (Math.abs(driverController.getLeftX()) > OI.LEFT_X_DEADBAND) ? -driverController.getLeftX() : 0,
        () -> -driverController.getRightX(),
        () -> -driverController.getRightY())
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton rezero = new JoystickButton(driverController, XboxController.Button.kX.value);
    rezero.whenPressed(new InstantCommand(drivebase::zeroGyro));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return new autoMove(drivebase);
  }
}
