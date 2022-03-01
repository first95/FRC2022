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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.commands.drivebase.AutoCollect;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ControlCargoHandling;
import frc.robot.commands.ControlClimber;
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
    // temp
    SmartDashboard.putString("AutoMove", "4Cargo");
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
    /*
    Update this whenever buttons are rebound to avoid double-binding a button
    Current Button Mappings:
      Driver:
        Left Stick up/down -> robot fwd/back
        Right Stick left/right -> robot turn
        Left Bumper -> Autocollect
        Right Bumper -> Airbrakes
        POV UP -> Auto shoot high hub
        POV DOWN -> Auto shoot low hub
        A -> Auto Climb 1
        B -> Auto Climb 2
        Y -> Auto Climb 3
        X -> Auto Climb 4
      Gunner:
        Left Trigger -> Run Collector
        Left Bumper -> Unspool climber winches
        Right Bumper -> Spool climber winches
        Y -> Manual shoot high
        B -> Manual cargo eject (via collector)
        A -> Toggle climber pneumatics
        X -> Deploy/undeploy (toggle) collector
	*/

    JoystickButton collectButton = new JoystickButton(oi.driverController, XboxController.Button.kLeftBumper.value);
    collectButton.whenHeld(new AutoCollect(drivebase, limelightcell));

    POVButton shootHigh = new POVButton(oi.driverController, 0);
    shootHigh.whenHeld(new AutoAim(true, drivebase, limelightport));

    POVButton shootLow = new POVButton(oi.driverController, 180);
    shootLow.whenHeld(new AutoAim(false, drivebase, limelightport));

    JoystickButton autoClimbS1 = new JoystickButton(oi.driverController, XboxController.Button.kA.value);
    autoClimbS1.whenHeld(new AutoClimbStage1(climber));

    JoystickButton autoClimbS2 = new JoystickButton(oi.driverController, XboxController.Button.kB.value);
    autoClimbS2.whenHeld(new AutoClimbStage2(climber));

    JoystickButton autoClimbS3 = new JoystickButton(oi.driverController, XboxController.Button.kY.value);
    autoClimbS3.whenHeld(new AutoClimbStage3(climber));

    JoystickButton autoClimbS4 = new JoystickButton(oi.driverController, XboxController.Button.kX.value);
    autoClimbS4.whenHeld(new AutoClimbStage4(climber));
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
    Trajectory currentTrajectory = new Trajectory();
    String JSONfile;
    int i = 0;
    Trajectory[] trajectoryList = new Trajectory[Constants.Auton.trajectoryFiles.length];
    for (String name : Constants.Auton.trajectoryFiles) {
      JSONfile = "paths/" + name + ".wpilib.json";
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
