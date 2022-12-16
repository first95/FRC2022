// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.swerveBase;

/** An example command that uses an example subsystem. */
public class AbsoluteDrive extends CommandBase {
  private swerveBase swerve;
  private PIDController thetaController;
  private DoubleSupplier vX, vY, heading;
  private double omega;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AbsoluteDrive(swerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController = new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    omega = thetaController.calculate(swerve.getYaw().getRadians(), Math.toRadians(heading.getAsDouble())) * Drivebase.MAX_ANGULAR_VELOCITY;
    swerve.drive(
      new Translation2d(
        vX.getAsDouble() * Drivebase.MAX_SPEED,
        vY.getAsDouble() * Drivebase.MAX_SPEED),
      omega,
      true,
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
