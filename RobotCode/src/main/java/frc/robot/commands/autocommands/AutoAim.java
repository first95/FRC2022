/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.OI;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoAim extends CommandBase {

  private double headingLastError = 0;
  private double headingIntegral = 0;
  private double rangeLastError = 0;
  private double rangeIntegral = 0;
  private double desiredDistance = 81;
  private boolean onTarget = false;
  private long targetAquiredtime = 0;

  private DriveBase m_DriveBase;
  private LimeLight m_LimeLight;

  public AutoAim(DriveBase driveBase, LimeLight limeLight, double desiredDistance) {
    m_DriveBase = driveBase;
    m_LimeLight = limeLight;
    Subsystem[] subsystems = {driveBase, limeLight};

    addRequirements(subsystems);
    this.desiredDistance = desiredDistance;
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
    double rangeLeft = 0;
    double rangeRight = 0;
    double left = 0;
    double right = 0;
    double headingErrorPercent = 0;
    double rangeErrorPercent = 0;

    boolean headingOnTarget = false;
    boolean rangeOnTarget = false;

    double headingError = m_LimeLight.getTX();
    double rangeError = m_LimeLight.getFloorDistanceToTarg() - desiredDistance;
    double targetValid = m_LimeLight.getTV();

    double headingProportional = 0;
    double headingDerivitive = 0;
    double headingRawCorrection = 0;
    double headingkp = SmartDashboard.getNumber("Vision heading Kp", 1);
    double headingki = SmartDashboard.getNumber("Vision heading Ki", 0);
    double headingkd = SmartDashboard.getNumber("Vision heading Kd", 0);

    double rangeProportional = 0;
    double rangeDerivitive = 0;
    double rangeRawCorrection = 0;
    double rangekp = SmartDashboard.getNumber("Vision range Kp", 2.5);
    double rangeki = SmartDashboard.getNumber("Vision range Ki", 0);
    double rangekd = SmartDashboard.getNumber("Vision range Kd", 5);

    if (targetValid == 1 && !onTarget) {
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
      if (Math.abs(rangeError) > Constants.VISION_RANGE_TOLERANCE_INCH) {
        rangeErrorPercent = (rangeError / desiredDistance);
        rangeProportional = -rangeErrorPercent;
        rangeIntegral = rangeErrorPercent + rangeIntegral;
        rangeDerivitive = rangeErrorPercent - rangeLastError;
        rangeRawCorrection = Math.max(Math.min((rangeProportional * rangekp) + (rangeIntegral * rangeki) + (rangeDerivitive * rangekd), Constants.VISION_RANGE_MAX_SPEED_PERCENT), -Constants.VISION_RANGE_MAX_SPEED_PERCENT);
        if (Math.abs(rangeRawCorrection) < Constants.VISION_RANGE_MIN_SPEED_PERCENT) {
          rangeRight = Math.copySign(Constants.VISION_RANGE_MIN_SPEED_PERCENT, rangeRawCorrection);
        }
        else {
          rangeRight = rangeRawCorrection;
        }
        rangeLeft = rangeRight;
        rangeOnTarget = false;
      }
      else {
        rangeLeft = 0;
        rangeRight = 0;
        rangeOnTarget = true;
      }
    }
    else if (onTarget) {
      if (desiredDistance == Constants.VISION_RANGE_A_INCH) {
        OI.auto_shooting_speed = 2100;
      }
      else if (desiredDistance == Constants.VISION_RANGE_B_INCH) {
        OI.auto_shooting_speed = 2300;
      }
      else if (desiredDistance == Constants.VISION_RANGE_C_INCH) {
        OI.auto_shooting_speed = 2650;
      }
      else if (desiredDistance == Constants.VISION_RANGE_D_INCH) {
        OI.auto_shooting_speed = 2800;
      }
      if (targetAquiredtime == 0) {
        targetAquiredtime = System.currentTimeMillis();
      }
      OI.auto_shooting = true;
      headingOnTarget = true;
      rangeOnTarget = true;
    }

    onTarget = headingOnTarget && rangeOnTarget;
    left = headingLeft + rangeLeft;
    right = headingRight + rangeRight;
    headingLastError = headingErrorPercent;
    rangeLastError = rangeErrorPercent; 
    m_DriveBase.driveWithTankControls(left, right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (targetAquiredtime != 0) {
      return (System.currentTimeMillis() > (3000 + targetAquiredtime));
    }
    else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_DriveBase.driveWithTankControls(0, 0);
    OI.auto_shooting = false;
    onTarget = false;
    
  }
}
