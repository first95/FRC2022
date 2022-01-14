/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PowerCellMover;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoSpinUpShooter extends CommandBase {
  private long startTime;
  private long timeOutMs;

  private PowerCellMover m_PowerCellMover;

  public AutoSpinUpShooter(PowerCellMover powerCellMover, long timeOutMs) {
    m_PowerCellMover = powerCellMover;
    this.timeOutMs = timeOutMs;
    // Use requires() here to declare subsystem dependencies
    addRequirements(powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_PowerCellMover.runShooterOpen(0.385);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return ((startTime + timeOutMs) < System.currentTimeMillis() );
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
