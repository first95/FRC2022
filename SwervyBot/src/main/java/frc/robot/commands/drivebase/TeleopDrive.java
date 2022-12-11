// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.swerveBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private swerveBase swerve;
  private DoubleSupplier vX, vY, omega;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(swerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("vX", vX.getAsDouble() * Drivebase.MAX_SPEED);
    SmartDashboard.putNumber("vY", vY.getAsDouble() * Drivebase.MAX_SPEED);
    SmartDashboard.putNumber("omega", omega.getAsDouble() * Drivebase.MAX_ANGULAR_VELOCITY);
    swerve.drive(
      new Translation2d(
        vX.getAsDouble(),
        vY.getAsDouble()).times(Drivebase.MAX_SPEED),
      omega.getAsDouble() * Drivebase.MAX_ANGULAR_VELOCITY,
      false,
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
