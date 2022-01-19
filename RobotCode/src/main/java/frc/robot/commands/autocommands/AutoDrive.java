/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

/**
 * Auto Drice for specified number of miliseconds at specified percent speed
 */
public class AutoDrive extends CommandBase {
  private long startTime;
  private long timeOutMs;
  private double speed;

  private DriveBase m_DriveBase;

  public AutoDrive(DriveBase driveBase, long timeOutMs, double speed) {
    this.timeOutMs = timeOutMs;
    this.speed = speed;

    m_DriveBase = driveBase;

    // Use requires() here to declare subsystem dependencies
    addRequirements(driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // m_DriveBase.driveWithTankControls(speed, speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return ((startTime + timeOutMs) < System.currentTimeMillis());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // m_DriveBase.driveWithTankControls(0, 0);
  }
}
