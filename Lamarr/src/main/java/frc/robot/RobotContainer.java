// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.commands.drivebase.AutoCollect;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControlCargoHandling;
import frc.robot.commands.ControlClimber;
import frc.robot.commands.PathTest;
import frc.robot.commands.autocommands.AutoMoves;
import frc.robot.commands.climber.AutoClimbStage1;
import frc.robot.commands.climber.AutoClimbStage2;
import frc.robot.commands.climber.AutoClimbStage3;
import frc.robot.commands.climber.AutoClimbStage4;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final OI oi = new OI();

  public DriverStation.Alliance teamAlliance;

  // The robot's subsystems and commands are defined here //
  public final DriveBase drivebase = new DriveBase();
  public final CargoHandler cargoHandler = new CargoHandler();
  public final Climber climber = new Climber();
  private final ControlCargoHandling controlCargoHandling = new ControlCargoHandling(cargoHandler);
  private final ManuallyControlDrivebase manuallyControlDrivebase = new ManuallyControlDrivebase(drivebase);
  private final ControlClimber controlClimber = new ControlClimber(climber);
  private final LimeLight limelightport = new LimeLight("port");
  private final LimeLight limelightcell = new LimeLight("cell");

  // Import trajectories
  public Trajectory[] trajectories = importTrajectories();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set the alliance from FMS
    setAlliance();

    // Set default commands
    drivebase.setDefaultCommand(manuallyControlDrivebase);
    cargoHandler.setDefaultCommand(controlCargoHandling);
    climber.setDefaultCommand(controlClimber);
  }

  public void setAlliance() {
    teamAlliance = DriverStation.getAlliance();
    limelightcell.SetTeamColor(teamAlliance);
    cargoHandler.setAlliance(teamAlliance);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton collectButton = new JoystickButton(oi.driverController, XboxController.Button.kLeftBumper.value);
    collectButton.whenHeld(new AutoCollect(drivebase, limelightcell));

    JoystickButton aimingButton = new JoystickButton(oi.driverController, XboxController.Button.kY.value);
    aimingButton.whenHeld(new AutoAim(true, drivebase, limelightport));

    JoystickButton lowButton = new JoystickButton(oi.driverController, XboxController.Button.kA.value);
    lowButton.whenHeld(new AutoAim(false, drivebase, limelightport));

    /*JoystickButton autoClimbS1 = new JoystickButton(oi.driverController, XboxController.Button.kA.value);
    autoClimbS1.whenHeld(new AutoClimbStage1(climber));

    JoystickButton autoClimbS2 = new JoystickButton(oi.driverController, XboxController.Button.kB.value);
    autoClimbS2.whenHeld(new AutoClimbStage2(climber));

    JoystickButton autoClimbS3 = new JoystickButton(oi.driverController, XboxController.Button.kY.value);
    autoClimbS3.whenHeld(new AutoClimbStage3(climber));

    JoystickButton autoClimbS4 = new JoystickButton(oi.driverController, XboxController.Button.kX.value);
    autoClimbS4.whenHeld(new AutoClimbStage4(climber));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutoMoves(drivebase, limelightport, trajectories);
  }

  public Trajectory[] importTrajectories() {
    Path trajectoryPath;
    Trajectory getCargo = new Trajectory();
    String getCargoJSON = "paths/TarmacToFirstCargo.wpilib.json";
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(getCargoJSON);
      getCargo = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + getCargoJSON, ex.getStackTrace());
    }
    Trajectory goShoot1 = new Trajectory();
    String goShoot1JSON = "paths/GoShoot1.wpilib.json";
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goShoot1JSON);
      goShoot1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + goShoot1JSON, ex.getStackTrace());
    }
    Trajectory next2cargo = new Trajectory();
    String next2cargoJSON = "paths/Next2Cargo.wpilib.json";
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(next2cargoJSON);
      next2cargo = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + next2cargoJSON, ex.getStackTrace());
    }
    Trajectory goShoot2 = new Trajectory();
    String goShoot2JSON = "paths/GoShoot2.wpilib.json";
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goShoot2JSON);
      goShoot2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + goShoot2JSON, ex.getStackTrace());
    }

    Trajectory[] trajectoryList = new Trajectory[4];
    trajectoryList[0] = getCargo;
    trajectoryList[1] = goShoot1;
    trajectoryList[2] = next2cargo;
    trajectoryList[3] = goShoot2;
    return trajectoryList;
  }
}
