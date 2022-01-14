/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;

/**
 * An example command. You can replace me with your own command.
 */
public class PositionCloseLoopControlCommand extends CommandBase {

  private DriveBase m_DriveBase;

  public PositionCloseLoopControlCommand(DriveBase driveBase) {
    m_DriveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    super.initialize();

    // Read Set Rotations from SmartDashboard
    double rotations = SmartDashboard.getNumber("SetPoint", 0);

    m_DriveBase.travleDistance(rotations);

    // Display set point (in rotations) on SmartDashboard
    SmartDashboard.putNumber("SetPoint", rotations);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
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
