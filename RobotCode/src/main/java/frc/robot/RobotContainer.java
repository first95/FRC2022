// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.OI.Controller;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.commands.vision.SetCameraMode;
import frc.robot.commands.vision.ToggleCameraMode;
import frc.robot.oi.JoystickAxisButton;
import frc.robot.oi.XBox360Controller;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionCoprocessor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final OI oi = new OI();

  // Robot subsystems
  public final VisionCoprocessor visionCoprocessor = new VisionCoprocessor();
  public final DriveBase driveBase = new DriveBase(oi);

  // Robot Commands
  public final SetCameraMode setCameraMode = new SetCameraMode(visionCoprocessor, false);
  public final ManuallyControlDrivebase manuallyControlDrivebase = new ManuallyControlDrivebase(driveBase);
  public final ToggleCameraMode toggleCameraMode = new ToggleCameraMode(visionCoprocessor);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveBase.initDefaultCommand(manuallyControlDrivebase);
    visionCoprocessor.initDefaultCommand(setCameraMode);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    JoystickButton cameraViewSwitcher = new JoystickButton(oi.driverController, OI.SWITCH_CAM_VIEW_BUTTON);
    cameraViewSwitcher.whenPressed(toggleCameraMode);
    // cameraViewSwitcher.close(); // Don't need this one anymore?

    JoystickAxisButton driverRumblerLeft = new JoystickAxisButton(oi.driverController,
        XBox360Controller.Axis.LEFT_TRIGGER.Number());
    driverRumblerLeft.whenPressed(new RumbleCommand(oi, Controller.DRIVER, Joystick.RumbleType.kLeftRumble, 1, 1.0));

    JoystickAxisButton driverRumblerRight = new JoystickAxisButton(oi.driverController,
        XBox360Controller.Axis.RIGHT_TRIGGER.Number());
    driverRumblerRight.whenPressed(new RumbleCommand(oi, Controller.DRIVER, Joystick.RumbleType.kRightRumble, 1, 1.0));

    JoystickAxisButton weaponsRumblerLeft = new JoystickAxisButton(oi.weaponsController,
        XBox360Controller.Axis.LEFT_TRIGGER.Number());
    weaponsRumblerLeft.whenPressed(new RumbleCommand(oi, Controller.WEAPONS, Joystick.RumbleType.kLeftRumble, 1, 1.0));

    JoystickAxisButton weaponsRumblerRight = new JoystickAxisButton(oi.weaponsController,
        XBox360Controller.Axis.RIGHT_TRIGGER.Number());
    weaponsRumblerRight
        .whenPressed(new RumbleCommand(oi, Controller.WEAPONS, Joystick.RumbleType.kRightRumble, 1, 1.0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
