// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoPowerCellMover;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PowerCellMover;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autocommands.AutoMoves;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final OI oi = new OI();

  // The robot's subsystems and commands are defined here...
  public final DriveBase drivebase = new DriveBase();
  private final PowerCellMover powerCellMover = new PowerCellMover();
  private final LimeLight limelightport = new LimeLight("port");
  private final LimeLight limelightcell = new LimeLight("cell");

  private final ManuallyControlDrivebase manuallyControlDrivebase = new ManuallyControlDrivebase(drivebase);
  private final AutoPowerCellMover autoPowerCellMover = new AutoPowerCellMover(powerCellMover);

  //import trajectories
  public Trajectory trajectories = importTrajectories();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands
    drivebase.setDefaultCommand(manuallyControlDrivebase);
    powerCellMover.setDefaultCommand(autoPowerCellMover);
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
    return new AutoMoves(drivebase, trajectories);
  }

  public Trajectory importTrajectories() {
    Path trajectoryPath;
    Trajectory getCargo = new Trajectory();
    String getCargoJSON = "paths/TarmacToFirstCargo.wpilib.json";
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(getCargoJSON);
      getCargo = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + getCargoJSON, ex.getStackTrace());
    }
    //Trajectory goShoot1 = new Trajectory();
    //String goShoot1JSON = "paths/GoShoot1.wpilib.json";
    //try {
    //  trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goShoot1JSON);
    //  goShoot1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //}
    //catch (IOException ex) {
    //  DriverStation.reportError("Unable to open trajectory: " + goShoot1JSON, ex.getStackTrace());
    //}
    //Trajectory [] trajectoryList = new Trajectory [2];
    //trajectoryList[0] = getCargo;
    //trajectoryList[1] = goShoot1;
    return getCargo;
  }
}
