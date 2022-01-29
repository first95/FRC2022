// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.OI.Controller;
import frc.robot.subsystems.DriveBase;

public class CollectColoredBall extends CommandBase {
  private final DriveBase m_drivebase;
  private final LimeLight m_limelight;
  private double headingLastError = 0;
  private double headingIntegral = 0;
  private boolean onTarget = false;

  public CollectColoredBall(DriveBase drivebase, LimeLight limelight) {
    m_drivebase = drivebase;
    m_limelight = limelight;

    Subsystem[] subsystems = { drivebase, limelight };
    addRequirements(subsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingLeft = 0;
    double headingRight = 0;
    double left = 0;
    double right = 0;
    double backupSpeed = 0;
    double headingErrorPercent = 0;

    boolean headingOnTarget = false;

    double headingError = m_limelight.getTX();
    double targetValid = m_limelight.getTV();

    double headingProportional = 0;
    double headingDerivitive = 0;
    double headingRawCorrection = 0;
    double headingkp = SmartDashboard.getNumber("Vision heading Kp", 1);
    double headingki = SmartDashboard.getNumber("Vision heading Ki", 0);
    double headingkd = SmartDashboard.getNumber("Vision heading Kd", 0);

    if (m_limelight.getDistanceToTarg() < 15)
      onTarget = true;

    if (targetValid == 1) {
      if (Math.abs(headingError) > Constants.VISION_HEADING_TOLERANCE_DEG) {
        headingErrorPercent = (headingError / Constants.VISION_CAM_FOV_X_DEG);
        headingProportional = headingErrorPercent;
        headingIntegral = headingErrorPercent + headingIntegral;
        headingDerivitive = headingErrorPercent - headingLastError;
        headingRawCorrection = Math.max(Math.min(
            (headingProportional * headingkp) + (headingIntegral * headingki) + (headingDerivitive * headingkd),
            Constants.VISION_HEADING_MAX_SPEED_PERCENT), -Constants.VISION_HEADING_MAX_SPEED_PERCENT);
        if (Math.abs(headingRawCorrection) < Constants.VISION_HEADING_MIN_SPEED_PERCENT) {
          headingRight = Math.copySign(Constants.VISION_HEADING_MIN_SPEED_PERCENT, headingRawCorrection);
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
      if (headingOnTarget || onTarget) {
        onTarget = true;
        backupSpeed = 0.3;
        RobotContainer.oi.auto_collect_speed = 1;
      }
    } else {
      // RobotContainer.oi.Rumble(Controller.DRIVER, RumbleType.kLeftRumble, 1.0,
      // 0.25);
    }

    left = headingLeft + backupSpeed;
    right = headingRight + backupSpeed;
    headingLastError = headingErrorPercent;
    m_drivebase.driveWithTankControls(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.driveWithTankControls(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
