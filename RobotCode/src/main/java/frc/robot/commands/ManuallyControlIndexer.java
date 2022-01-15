/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * An example command. You can replace me with your own command.
 */
public class ManuallyControlIndexer extends CommandBase {
  public static double MANUAL_RUN_SPEED = 0.8;

  public ManuallyControlIndexer() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.indexer);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (RobotContainer.oi.getRunIndexer()) {
      // Robot.indexer.runIndexer(MANUAL_RUN_SPEED);
    } else {
      // Robot.indexer.runIndexer(0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
