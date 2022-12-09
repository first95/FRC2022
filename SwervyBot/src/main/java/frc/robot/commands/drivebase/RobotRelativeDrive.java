// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import frc.robot.subsystems.swerveBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RobotRelativeDrive extends CommandBase {
  private final swerveBase driveBase;
  private DoubleSupplier vX, vY, omega;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RobotRelativeDrive(swerveBase subsystem, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
    driveBase = subsystem;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.setVelocity(new ChassisSpeeds(
      vX.getAsDouble(),
      vY.getAsDouble(),
      omega.getAsDouble()
      ));
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
