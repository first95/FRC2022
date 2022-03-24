/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Vision;
import frc.robot.Constants;
import frc.robot.OI.Controller;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command. You can replace me with your own command.
 */
public class AutoAim extends CommandBase {

  private DriveBase drivebase;
  private LimeLight limelightport;
  private boolean stableYawRate; // Wait for a stable yaw rate from the IMU before shooting, to prevent overshoot
  private double previousYaw;

  private double headingLastError, headingIntegral, headingLeft, headingRight, headingErrorPercent,
      headingProportional, headingDerivitive, headingRawCorrection, headingkp, headingki, headingkd,
      headingError;
  private double rangeLastError, rangeIntegral, rangeLeft, rangeRight, rangeErrorPercent,
      rangeProportional, rangeDerivitive, rangeRawCorrection, rangekp, rangeki, rangekd,
      rangeError;
  private double left, right, targetValid, range;
  private boolean onTarget, headingOnTarget, rangeOnTarget, highHub;

  /**
   * Aims at and then shoots into one of the two hubs (upper or lower) with
   * airbrakes
   * engaged.
   * 
   * @param highHub       True for high hub, false for low hub.
   * @param drivebase     Self-Explanatory
   * @param limelightport The limelight tracking the hub target
   */
  public AutoAim(boolean highHub, DriveBase drivebase, LimeLight limelightport) {
    this.limelightport = limelightport;
    this.drivebase = drivebase;
    this.highHub = highHub;
    addRequirements(drivebase);
    addRequirements(limelightport);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    previousYaw = drivebase.imu.getYaw();
    headingOnTarget = false;
    rangeOnTarget = false;
    onTarget = false;
    headingIntegral = 0;
    rangeIntegral = 0;
    headingLastError = 0;
    rangeLastError = 0;

    headingkp = Vision.HEADING_KP;
    headingki = Vision.HEADING_KI;
    headingkd = Vision.HEADING_KD;

    rangekp = Vision.RANGE_KP;
    rangeki = Vision.RANGE_KI;
    rangekd = Vision.RANGE_KD;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    range = limelightport.getFloorDistanceToTarg();
    headingError = limelightport.getTX();
    rangeError = range - Vision.DESIRED_RANGE_INCH;
    targetValid = limelightport.getTV();
    RobotContainer.oi.auto_shoot_pre_spool = true;

    RobotContainer.oi.auto_shooting_speed = highHub ? CargoHandler.distanceToShooterRPM(range)
        : Constants.CargoHandling.SHOOTING_LOW_SPEED;
    RobotContainer.oi.auto_roller_speed = highHub
        ? CargoHandler.distanceToShooterRPM(range) * Constants.CargoHandling.SHOOTER_RATIO
        : Constants.CargoHandling.ROLLER_LOW_SPEED;

    if (targetValid == 1 && !onTarget) {
      if (Math.abs(headingError) > Vision.HEADING_TOLERANCE_DEG) {
        headingErrorPercent = (headingError / Vision.CAM_FOV_X_DEG);
        headingProportional = headingErrorPercent;
        headingIntegral = headingErrorPercent + headingIntegral;
        headingDerivitive = headingErrorPercent - headingLastError;
        headingRawCorrection = Math.max(
            Math.min(
                (headingProportional * headingkp) + (headingIntegral * headingki) + (headingDerivitive * headingkd),
                Vision.HEADING_MAX_SPEED_PERCENT),
            -Vision.HEADING_MAX_SPEED_PERCENT);

        if (Math.abs(headingRawCorrection) < Vision.HEADING_MIN_SPEED_PERCENT) {
          headingRight = Math.copySign(Vision.HEADING_MIN_SPEED_PERCENT, headingRawCorrection);
        } else {
          headingRight = headingRawCorrection;
        }
        headingLeft = -headingRight;
        headingOnTarget = false;
      } else {
        headingLeft = 0;
        headingRight = 0;
        headingOnTarget = true;
      }

      if (Math.abs(rangeError) > Vision.RANGE_TOLERANCE_INCH) {
        rangeErrorPercent = rangeError / 20;
        rangeProportional = rangeErrorPercent;
        rangeIntegral = rangeErrorPercent + rangeIntegral;
        rangeDerivitive = rangeErrorPercent - rangeLastError;
        rangeRawCorrection = (rangeProportional * rangekp) + (rangeIntegral * rangeki) + (rangeDerivitive * rangekd);

        if (Math.abs(rangeRawCorrection) < Vision.RANGE_MIN_SPEED_PERCENT) {
          rangeRight = Math.copySign(Vision.RANGE_MIN_SPEED_PERCENT, rangeRawCorrection);
        } else {
          rangeRight = -rangeRawCorrection;
        }
        rangeLeft = rangeRight;
        rangeOnTarget = false;
      } else {
        rangeLeft = 0;
        rangeRight = 0;
        rangeOnTarget = true;
      }
    } else if (onTarget) {
      //if (stableYawRate) {
        drivebase.setAirBrakes(true);
        RobotContainer.oi.auto_shooting = true;
        headingOnTarget = true;
        rangeOnTarget = true;
      //}
    } else {
      RobotContainer.oi.Rumble(Controller.DRIVER, RumbleType.kLeftRumble, 1.0, 0.25);
    }

    // SmartDashboard.putNumber("YAW Threshold", 10);
    SmartDashboard.putNumber("Current Yaw", drivebase.imu.getYaw());
    SmartDashboard.putNumber("Previous Yaw", previousYaw);
    SmartDashboard.putBoolean("Is Yaw Stable?", stableYawRate);
    // double manualYawThreshold = SmartDashboard.getNumber("YAW Threshold", 10);

    // stableYawRate = previousYaw + manualYawThreshold < drivebase.imu.getYaw()
    // && previousYaw - manualYawThreshold > drivebase.imu.getYaw();
    stableYawRate = Math.abs(previousYaw - drivebase.imu.getYaw()) < Constants.CargoHandling.YAW_THRESHOLD;

    previousYaw = drivebase.imu.getYaw();
    onTarget = headingOnTarget && rangeOnTarget;
    left = headingLeft + rangeLeft;
    right = headingRight + rangeRight;
    headingLastError = headingErrorPercent;
    rangeLastError = rangeErrorPercent;
    drivebase.driveWithTankControls(left, right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.driveWithTankControls(0, 0);
    drivebase.setAirBrakes(false);
    RobotContainer.oi.auto_shoot_pre_spool = false;
    RobotContainer.oi.auto_shooting = false;
    RobotContainer.oi.auto_shooting_speed = 0;
    onTarget = false;

  }
}
