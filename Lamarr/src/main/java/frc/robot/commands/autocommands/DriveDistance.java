// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.DriveBase;

public class DriveDistance extends CommandBase {
  private DriveBase drivebase;
  private double distance, speed, targDist, driveSpeed, start, currentDistance;
  private boolean ramp;

  /** Drives at the given speed for a number of inches
   * @param distance Distance to drive, in inches.  Always positive.
   * @param speed -1 to 1; speed to drive as percentage.  Negative for reverse.
  */
  public DriveDistance(double distance, double speed, DriveBase drivebase) {
    this.drivebase = drivebase;
    this.distance = distance;
    this.speed = speed;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (speed <= Auton.MAX_SPEED_BEFORE_RAMP) {
      ramp = false;
    } else {
      ramp = true;
    }

    targDist = distance * Constants.Universal.METERS_PER_INCH;
    start = drivebase.getWheelPositions()[0];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = (drivebase.getWheelPositions()[0] - start) / Constants.Universal.METERS_PER_INCH;
    if (ramp) {
      driveSpeed = speedRamp(currentDistance, distance, speed);
      System.out.println(driveSpeed);
    } else {
      driveSpeed = speed;
    }

    drivebase.driveWithTankControls(-driveSpeed, -driveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.driveWithTankControls(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drivebase.getWheelPositions()[0] >= targDist) ? true : false;
  }

  /**
   * Ramps speed using a sine function based on distance.
   * Explore the math here: https://www.desmos.com/calculator/noolwcpqoo
   * @param currentDistance Current distance traveled, in inches
   * @param maxDistance Total distance to travel, in inches
   * @param maxSpeed Target speed to ramp to.  -1 to 1.
   * @return Speed to drive at; -1 to 1
   */
  private double speedRamp(double currentDistance, double maxDistance, double maxSpeed) {
    double sign = Math.copySign(1, maxSpeed);
    double minSpeed = Auton.MIN_DRIVE_SPEED * sign;
    double distanceAdjustment = Math.PI / (2 * Auton.RAMP_UP_END * maxDistance);
    currentDistance = Math.abs(currentDistance);
    if (currentDistance <= Auton.RAMP_UP_END * maxDistance) {
      return (maxSpeed - minSpeed) * Math.sin(currentDistance * distanceAdjustment) + minSpeed;
    } else if (currentDistance < Auton.RAMP_DOWN_START * maxDistance) {
      return maxSpeed;
    } else {
      return (maxSpeed - minSpeed) * Math.sin(distanceAdjustment * (currentDistance - 
        (maxDistance * (Auton.RAMP_DOWN_START - Auton.RAMP_UP_END)))) + minSpeed;
    }
  }
}
