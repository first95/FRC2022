// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.commands.vision.SetCameraMode;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionCoprocessor;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final VisionCoprocessor m_visionCoprocessor = new VisionCoprocessor();
  private final DriveBase m_driveBase = new DriveBase();

  public final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final SetCameraMode m_setCameraMode = new SetCameraMode(m_visionCoprocessor, false);
  public final ManuallyControlDrivebase m_manuallyControlDrivebase = new ManuallyControlDrivebase(m_driveBase);
  
  public static final OI oi = new OI();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveBase.initDefaultCommand(m_manuallyControlDrivebase);
    m_visionCoprocessor.initDefaultCommand(m_setCameraMode);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
