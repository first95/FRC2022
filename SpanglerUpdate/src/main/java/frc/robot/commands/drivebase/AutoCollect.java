/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI.Controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoCollect extends CommandBase {

  private double headingLastError = 0;
  private double headingIntegral = 0;
  private boolean onTarget = false;
  private final DriveBase drivebase;
  private final LimeLight limelightcell;

  public AutoCollect(DriveBase drivebase_param, LimeLight limelightcell_param) {
    drivebase = drivebase_param;
    limelightcell = limelightcell_param;
    addRequirements(drivebase);
    addRequirements(limelightcell);
    limelightcell.setVisionPipeline(0);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double headingLeft = 0;
    double headingRight = 0;
    double left = 0;
    double right = 0;
    double backupSpeed = 0;
    double headingErrorPercent = 0;

    boolean headingOnTarget = false;

    double headingError = limelightcell.getTX();
    double targetValid = limelightcell.getTV();

    double headingProportional = 0;
    double headingDerivitive = 0;
    double headingRawCorrection = 0;
    double headingkp = SmartDashboard.getNumber("Vision heading Kp", 1);
    double headingki = SmartDashboard.getNumber("Vision heading Ki", 0);
    double headingkd = SmartDashboard.getNumber("Vision heading Kd", 0);

    if (targetValid == 1) {
      if (Math.abs(headingError) > Constants.VISION_HEADING_TOLERANCE_DEG) {
        headingErrorPercent = (headingError / Constants.VISION_CAM_FOV_X_DEG);
        headingProportional = headingErrorPercent;
        headingIntegral = headingErrorPercent + headingIntegral;
        headingDerivitive = headingErrorPercent - headingLastError;
        headingRawCorrection = Math.max(Math.min((headingProportional * headingkp) + (headingIntegral * headingki) + (headingDerivitive * headingkd), Constants.VISION_HEADING_MAX_SPEED_PERCENT), -Constants.VISION_HEADING_MAX_SPEED_PERCENT);
        if (Math.abs(headingRawCorrection) < Constants.VISION_HEADING_MIN_SPEED_PERCENT) {
          headingRight = Math.copySign(Constants.VISION_HEADING_MIN_SPEED_PERCENT, headingRawCorrection);
        }
        else {
          headingRight = headingRawCorrection;
        }
        headingLeft = -headingRight;
        headingOnTarget = false;
      }
      else {
        headingLeft = 0;
        headingRight = 0;
        headingOnTarget = true;
      }
      if (headingOnTarget || onTarget) {
        onTarget = true;
        backupSpeed = 0.3;
        RobotContainer.oi.auto_collect_speed = 1;
      }
    }
    else {
      RobotContainer.oi.Rumble(Controller.DRIVER, RumbleType.kLeftRumble, 1.0, 0.25);
    }

    left = headingLeft + backupSpeed;
    right = headingRight + backupSpeed;
    headingLastError = headingErrorPercent; 
    drivebase.driveWithTankControls(left, right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivebase.driveWithTankControls(0, 0);
    onTarget = false;
    RobotContainer.oi.auto_collect_speed = 0;
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
    drivebase.driveWithTankControls(0, 0);
    onTarget = false;
    RobotContainer.oi.auto_collect_speed = 0;
  }
}
