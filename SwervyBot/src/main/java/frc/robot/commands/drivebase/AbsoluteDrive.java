// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.OI;
import frc.robot.subsystems.SwerveBase;

/** An example command that uses an example subsystem. */
public class AbsoluteDrive extends CommandBase {
  private SwerveBase swerve;
  private PIDController thetaController;
  private DoubleSupplier vX, vY, headingX, headingY;
  private double omega, angle, lastAngle, x, y;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AbsoluteDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingX, DoubleSupplier headingY) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingX = headingX;
    this.headingY = headingY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController = new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    lastAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerve.wasGyroReset()) {
      lastAngle = 0;
      swerve.clearGyroReset();
    }

    if (Math.hypot(headingX.getAsDouble(), headingY.getAsDouble()) < 0.5) {
      angle = lastAngle;
    } else {
      angle = Math.atan2(headingY.getAsDouble(), headingX.getAsDouble());
    }

    omega = thetaController.calculate(swerve.getYaw().getRadians(), angle) * Drivebase.MAX_ANGULAR_VELOCITY;
    x = Math.pow(vX.getAsDouble(), 3) * Drivebase.MAX_SPEED;
    y = Math.pow(vY.getAsDouble(), 3) * Drivebase.MAX_SPEED;

    swerve.drive(
      new Translation2d(x, y),
      omega,
      true,
      true);
    
    lastAngle = angle;
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
