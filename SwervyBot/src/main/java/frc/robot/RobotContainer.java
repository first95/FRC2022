// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OI;
import frc.robot.commands.autoCommands.autoMove;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  private SendableChooser<CommandBase> driveModeSelector;

  //XboxController driverController = new XboxController(OI.DRIVER_CONTROLLER_PORT);
  Joystick driverController = new Joystick(OI.DRIVER_CONTROLLER_PORT);
  Joystick rotationController = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
      AbsoluteDrive absoluteDrive = new AbsoluteDrive(
        drivebase,
        // Applies deadbands and inverts controls because joysticks are back-right positive while robot
        // controls are front-left positive
        () -> (Math.abs(driverController.getY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OI.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
        () -> -rotationController.getX(),
        () -> -rotationController.getY(), true);

      AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(
        drivebase,
        // Applies deadbands and inverts controls because joysticks are back-right positive while robot
        // controls are front-left positive
        () -> (Math.abs(driverController.getY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OI.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
        () -> -rotationController.getX(),
        () -> -rotationController.getY(), false);

      TeleopDrive openRobotRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OI.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
        () -> -driverController.getTwist(), () -> false, true);
      
      TeleopDrive closedRobotRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OI.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
        () -> -driverController.getTwist(), () -> false, false);
      
      TeleopDrive openFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OI.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
        () -> -driverController.getTwist(), () -> true, true);

      TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getY()) > OI.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OI.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
        () -> -driverController.getTwist(), () -> true, false);

      driveModeSelector = new SendableChooser<>();
      driveModeSelector.setDefaultOption("AbsoluteDrive", absoluteDrive);
      driveModeSelector.addOption("Field Relative", openFieldRel);
      driveModeSelector.addOption("Robot Relative", openRobotRel);
      driveModeSelector.addOption("Absolute (Closed)", closedAbsoluteDrive);
      driveModeSelector.addOption("Field Relative (Closed)", closedFieldRel);
      driveModeSelector.addOption("Robot Relative (Closed)", closedRobotRel);

      SmartDashboard.putData(driveModeSelector);

      
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton rezero = new JoystickButton(driverController, 1);
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

  public void setDriveMode() {
    drivebase.setDefaultCommand(driveModeSelector.getSelected());
  }
}
