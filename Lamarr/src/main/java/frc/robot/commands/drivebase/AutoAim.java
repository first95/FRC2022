/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CargoHandling;
import frc.robot.Constants.Vision;
import frc.robot.Constants;
import frc.robot.OI.Controller;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterHood;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command. You can replace me with your own command.
 */
public class AutoAim extends CommandBase {

  private DriveBase drivebase;
  private LimeLight limelightport;
  private ShooterHood shooterhood;

  private double headingLastError, headingIntegral, headingLeft, headingRight, headingErrorPercent,
      headingProportional, headingDerivitive, headingRawCorrection, headingkp, headingki, headingkd,
      headingError;
  private double rangeLastError, rangeIntegral, rangeLeft, rangeRight, rangeErrorPercent,
      rangeProportional, rangeDerivitive, rangeRawCorrection, rangekp, rangeki, rangekd,
      rangeError;
  private double left, right, targetValid, range, targetRange;
  private boolean onTarget, headingOnTarget, rangeOnTarget, highHub, far;

  /**
   * Aims at and then shoots into one of the two hubs (upper or lower) with
   * airbrakes
   * engaged.
   * 
   * @param highHub       True for high hub, false for low hub.
   * @param drivebase     Self-Explanatory
   * @param limelightport The limelight tracking the hub target
   */
  public AutoAim(boolean highHub, DriveBase drivebase, LimeLight limelightport, ShooterHood shooterhood) {
    this.limelightport = limelightport;
    this.drivebase = drivebase;
    this.shooterhood = shooterhood;
    this.highHub = highHub;
    addRequirements(drivebase);
    addRequirements(limelightport);
    addRequirements(shooterhood);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
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

    range = limelightport.getFloorDistanceToTarg();

    // HOOD DOWN at 137
    // HOOD UP at 138
    if (Math.abs(Vision.DESIRED_RANGE_INCH - range) < Math.abs(Vision.FAR_RANGE_INCH - range)) {
      targetRange = range; // Vision.DESIRED_RANGE_INCH;
      shooterhood.setHood(true);
      far = false;
    } else {
      targetRange = range; // Vision.FAR_RANGE_INCH;
      shooterhood.setHood(false);
      far = true;
    }

    // There is a range between ~105 - ~175 where we undershoot with the hood in and we
    // overshoot with the hood out. To fix this if we are 105 - 140 we drive forward to 105.
    // if we are 140 - 175 we drive backwards and extend the hood to 175.
    if(range > 105 && range <= 140)
      targetRange = 105;
    else if (range >= 141 && range < 175)
      targetRange = 175;
    else if(range > Constants.Vision.MAX_RANGE_INCH)
      targetRange = Constants.Vision.MAX_RANGE_INCH;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    range = limelightport.getFloorDistanceToTarg();
    headingError = limelightport.getTX();
    rangeError = range - targetRange;
    targetValid = limelightport.getTV();

    // Shooter RPM Correction for robot (based off distance to target)
    if (far) {
      RobotContainer.oi.auto_shooting_speed = CargoHandler.farDistanceToShooterRPM(range);
      RobotContainer.oi.auto_roller_speed = CargoHandler.farDistanceToShooterRPM(range) *
        SmartDashboard.getNumber("Shooter Ratio", Constants.CargoHandling.SHOOTER_RATIO);
    } else {
      RobotContainer.oi.auto_shooting_speed = highHub ? CargoHandler.distanceToShooterRPM(range)
          : Constants.CargoHandling.SHOOTING_LOW_SPEED;
      RobotContainer.oi.auto_roller_speed = highHub
          ? CargoHandler.distanceToShooterRPM(range) * SmartDashboard.getNumber("Shooter Ratio", Constants.CargoHandling.SHOOTER_RATIO)
          : Constants.CargoHandling.ROLLER_LOW_SPEED;
    }

    // Position Correction for robot (distance + heading)
    if (targetValid == 1 && !onTarget) {
      // Heading correction
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

      // Range correction
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
        drivebase.setAirBrakes(true);
        RobotContainer.oi.auto_shooting = true;
        headingOnTarget = true;
        rangeOnTarget = true;
    } else {
      RobotContainer.oi.Rumble(Controller.DRIVER, RumbleType.kLeftRumble, 1.0, 0.25);
    }

    // Drive to correct range + heading
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
    RobotContainer.oi.auto_shooting = false;
    RobotContainer.oi.auto_shooting_speed = 0;
    RobotContainer.oi.auto_roller_speed = 0;
    onTarget = false;

  }
}
