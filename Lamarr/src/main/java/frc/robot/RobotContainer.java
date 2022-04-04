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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.commands.drivebase.AutoCollect;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterHood;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ControlCargoHandling;
import frc.robot.commands.ControlClimber;
import frc.robot.commands.climber.AutoClimbStage1;
import frc.robot.commands.climber.AutoClimbStage2;
import frc.robot.commands.climber.AutoClimbStage3;
import frc.robot.commands.climber.AutoClimbStage4;
import frc.robot.commands.climber.DefensiveMode;
import frc.robot.commands.climber.PassiveMode;

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
  public final ShooterHood shooterhood = new ShooterHood();
  private final ControlCargoHandling controlCargoHandling = new ControlCargoHandling(cargoHandler);
  private final ManuallyControlDrivebase manuallyControlDrivebase = new ManuallyControlDrivebase(drivebase);
  private final ControlClimber controlClimber = new ControlClimber(climber);
  public final LimeLight limelightport = new LimeLight("port");
  private final LimeLight limelightcell = new LimeLight("cell");

  // Import trajectories
  public Trajectory[] trajectories = importTrajectories();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands
    drivebase.setDefaultCommand(manuallyControlDrivebase);
    cargoHandler.setDefaultCommand(controlCargoHandling);
    climber.setDefaultCommand(controlClimber);
  }

  public void setAlliance(Alliance teamAlliance) {
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
    /*
    Update this whenever buttons are rebound to avoid double-binding a button
    Current Button Mappings:
      Driver:
        Left Stick up/down -> robot fwd/back
        Right Stick left/right -> robot turn
        Left Bumper -> Autocollect
        Right Bumper -> Airbrakes
        Y -> Auto shoot high hub
        A -> Auto shoot low hub
        POV DOWN -> Auto Climb 1
        POV RIGHT -> Auto Climb 2
        POV UP -> Auto Climb 3
        POV LEFT -> Auto Climb 4
      Gunner:
        Left Trigger -> Run Collector
        Right Trigger -> Manually Eject Cargo
        Left Bumper -> Unspool climber winches
        Right Bumper -> Spool climber winches
        Start -> Deploy climber to defensive mode
        Back -> Retract climber from defensive mode
        Y -> Manual shoot high
        B -> Manual shoot low
        A -> Toggle climber pneumatics
        X -> Deploy/undeploy (toggle) collector
	*/

    JoystickButton collectButton = new JoystickButton(oi.driverController, XboxController.Button.kLeftBumper.value);
    collectButton.whenHeld(new AutoCollect(drivebase, limelightcell));

    JoystickButton shootHigh = new JoystickButton(oi.driverController, XboxController.Button.kY.value);
    shootHigh.whenHeld(new AutoAim(true, drivebase, limelightport, shooterhood));

    JoystickButton shootLow = new JoystickButton(oi.driverController, XboxController.Button.kA.value);
    shootLow.whenHeld(new AutoAim(false, drivebase, limelightport, shooterhood));

    POVButton autoClimbS1 = new POVButton(oi.driverController, 180);
    autoClimbS1.whenHeld(new AutoClimbStage1(climber));

    POVButton autoClimbS2 = new POVButton(oi.driverController, 90);
    autoClimbS2.whenHeld(new AutoClimbStage2(climber));

    POVButton autoClimbS3 = new POVButton(oi.driverController, 0);
    autoClimbS3.whenHeld(new AutoClimbStage3(climber));

    POVButton autoClimbS4 = new POVButton(oi.driverController, 270);
    autoClimbS4.whenHeld(new AutoClimbStage4(climber));

    JoystickButton climberDefensiveMode = new JoystickButton(oi.weaponsController, XboxController.Button.kStart.value);
    climberDefensiveMode.whenHeld(new DefensiveMode(climber));

    JoystickButton climberPassiveMode = new JoystickButton(oi.weaponsController, XboxController.Button.kBack.value);
    climberPassiveMode.whenHeld(new PassiveMode(climber));

  }


  public Trajectory[] importTrajectories() {
    Path trajectoryPath;
    Trajectory currentTrajectory;
    String JSONfile;
    Trajectory[] trajectoryList = new Trajectory[Constants.Auton.trajectoryFiles.length];
    for (int i = 0; i < Constants.Auton.trajectoryFiles.length; i++) {
      currentTrajectory = new Trajectory();
      JSONfile = "paths/" + Constants.Auton.trajectoryFiles[i] + ".wpilib.json";
      try {
        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(JSONfile);
        currentTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + JSONfile, ex.getStackTrace());
      }
      trajectoryList[i] = currentTrajectory;
    }

    return trajectoryList;
  }
}
