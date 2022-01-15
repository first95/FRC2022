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
public class ManuallyControlShooter extends CommandBase {
  public static double MANUAL_RUN_SPEED = 0.3;
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double current_speed = 0;

  public ManuallyControlShooter() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Singulator speed setting
    if (RobotContainer.oi.getShooterButton()) {
      // Robot.shooter.runShooterOpen(MANUAL_RUN_SPEED);
      current_speed = MANUAL_RUN_SPEED;
    } else {
      // Slowing down motor and don't want to do it too fast
      if (current_speed < MIN_RUN_SPEED) {
        current_speed = 0;
      } else {
        current_speed = current_speed * MANUAL_RUN_SPEED;
      }
      // Robot.shooter.runShooterOpen(current_speed);
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
