// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoPowerCellMover;
import frc.robot.commands.autocommands.AutoMoves;
import frc.robot.subsystems.DriveBase.GearShiftMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public static double AutoDriveSpeed = 0;
  public static double PSAutoDriveSpeed = 0.2;
  public static boolean retractGroundCollectorDisabled;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Show what command your subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_robotContainer.drivebase);

    // Show git build information from Jar Manifest
    // TODO: Unable to find these details, results in a NULL exception
    // SmartDashboard.putString("BuildHost-BranchName",
    // Robot.class.getPackage().getImplementationTitle());
    // SmartDashboard.putString("GitCommitID-BuildTimestamp",
    // Robot.class.getPackage().getImplementationVersion());

    SmartDashboard.putNumber("Pre-Shoot Automode Drive speed", PSAutoDriveSpeed);
    SmartDashboard.putNumber("Automode Drive speed (neg for backwards)", AutoDriveSpeed);
    SmartDashboard.putNumber("Range 1 Power", 2100);
    SmartDashboard.putNumber("Range 2 Power", 2300);
    SmartDashboard.putNumber("Range 3 Power", 2900);
    SmartDashboard.putNumber("Range 4 Power", 3100);

    // Disable brakes on talons to make it
    // easier to push
    m_robotContainer.drivebase.brake(false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    RobotContainer.oi.visit();
    m_robotContainer.drivebase.visit();

    SmartDashboard.putBoolean("SingulatorOccupied", m_robotContainer.powerCellMover.getSingulatorSensor());
    SmartDashboard.putBoolean("IndexerEntranceOccupied", m_robotContainer.powerCellMover.getIndexerEntranceSensor());
    SmartDashboard.putBoolean("IndexerPosition1Occupied", m_robotContainer.powerCellMover.getIndexerLoadedSensor());
    SmartDashboard.putBoolean("ShooterLoaded", m_robotContainer.powerCellMover.getShooterLoadedSensor());

    SmartDashboard.putBoolean("Shooter is loaded", AutoPowerCellMover.shooterIsLoadedCheck);
    SmartDashboard.putBoolean("Moving into singulator", AutoPowerCellMover.movingFromSingulator);
    SmartDashboard.putBoolean("Moving into indexer", AutoPowerCellMover.movingIntoIndexer);
    SmartDashboard.putBoolean("Is in indexer", AutoPowerCellMover.isInIdexer);

    SmartDashboard.putNumber("Shooter speed (RPM)", m_robotContainer.powerCellMover.getShooterSpeed());

    // SmartDashboard.getNumber("Automode Drive speed (neg for backwards)",
    // AutoDriveSpeed);
    // SmartDashboard.putNumber("Pre-Shoot Automode Drive speed", PSAutoDriveSpeed);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.drivebase.brake(false);
    m_robotContainer.powerCellMover.deploy.set(false);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean("SingulatorOccupied", m_robotContainer.powerCellMover.getSingulatorSensor());
    SmartDashboard.putBoolean("IndexerEntranceOccupied", m_robotContainer.powerCellMover.getIndexerEntranceSensor());
    SmartDashboard.putBoolean("IndexerPosition1Occupied", m_robotContainer.powerCellMover.getIndexerLoadedSensor());
    SmartDashboard.putBoolean("ShooterLoaded", m_robotContainer.powerCellMover.getShooterLoadedSensor());
    SmartDashboard.putBoolean("dummy", AutoPowerCellMover.dummy);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Unlock the auto shifter
    // drivebase.setShiftMode(GearShiftMode.AUTOSHIFT);

    m_robotContainer.drivebase.brake(true);
    retractGroundCollectorDisabled = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
