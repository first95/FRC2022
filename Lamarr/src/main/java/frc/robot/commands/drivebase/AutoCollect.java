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
import frc.robot.Constants.Vision;
import frc.robot.OI.Controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoCollect extends CommandBase {

  private double headingLeft, headingRight, left, right, backupSpeed, headingErrorPercent,
    headingProportional, headingDerivitive, headingIntegral, headingLastError, headingRawCorrection;
  private boolean onTarget, headingOnTarget;
  private DriveBase drivebase;
  private LimeLight limelightcell;

  public AutoCollect(DriveBase drivebase, LimeLight limelightcell) {
    this.drivebase = drivebase;
    this.limelightcell = limelightcell;
    addRequirements(drivebase);
    addRequirements(limelightcell);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    headingLeft = 0;
    headingRight = 0;
    left = 0;
    right = 0;
    backupSpeed = 0;
    headingErrorPercent = 0;
    headingOnTarget = false;
    headingProportional = 0;
    headingDerivitive = 0;
    headingRawCorrection = 0;
    headingLastError = 0;
    headingIntegral = 0;
    onTarget = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double headingError = limelightcell.getTX();
    double targetValid = limelightcell.getTV();
    double headingkp = Vision.HEADING_KP;
    double headingki = Vision.HEADING_KI;
    double headingkd = Vision.HEADING_KD;

    if (targetValid == 1) {
      if (Math.abs(headingError) > Vision.HEADING_TOLERANCE_DEG) {
        headingErrorPercent = (headingError / Vision.CAM_FOV_X_DEG);
        headingProportional = headingErrorPercent;
        headingIntegral = headingErrorPercent + headingIntegral;
        headingDerivitive = headingErrorPercent - headingLastError;
        headingRawCorrection = Math.max(Math.min((headingProportional * headingkp) + (headingIntegral * headingki) + (headingDerivitive * headingkd), Vision.HEADING_MAX_SPEED_PERCENT), -Vision.HEADING_MAX_SPEED_PERCENT);
        if (Math.abs(headingRawCorrection) < Vision.HEADING_MIN_SPEED_PERCENT) {
          headingRight = Math.copySign(Vision.HEADING_MIN_SPEED_PERCENT, headingRawCorrection);
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
        backupSpeed = 0.6;
        RobotContainer.oi.auto_collect_speed = 0.6;
      }
    }
    else {
      RobotContainer.oi.Rumble(Controller.DRIVER, RumbleType.kLeftRumble, 1.0, 0.25);
      backupSpeed = 0;
      headingLeft = 0;
      headingRight = 0;
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
}
