// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class swerveBase extends SubsystemBase {
  // Swerve base kinematics object
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.Drivebase.FRONT_LEFT_X, Constants.Drivebase.FRONT_LEFT_Y),
    new Translation2d(Constants.Drivebase.FRONT_RIGHT_X, Constants.Drivebase.FRONT_RIGHT_Y),
    new Translation2d(Constants.Drivebase.BACK_LEFT_X, Constants.Drivebase.BACK_LEFT_Y),
    new Translation2d(Constants.Drivebase.BACK_RIGHT_X, Constants.Drivebase.BACK_RIGHT_Y)
  );

  // Current desired robot speed (initialized at 0, 0, 0)
  ChassisSpeeds robotVelocity = new ChassisSpeeds();

  // Drive motors
  private CANSparkMax driveFL = new CANSparkMax(deviceId, type)
  private CANSparkMax driveFR = new CANSparkMax(deviceId, type)
  private CANSparkMax driveBL = new CANSparkMax(deviceId, type)
  private CANSparkMax driveBR = new CANSparkMax(deviceId, type)  

  /** Creates a new swerve drivebase subsystem.  This will handle kinematics and
   * odometry. This also handles individual modual control; it will use ChassisSpeeds objects
   * given by commands to constantly update wheel position and speed.*/
  public swerveBase() {}

  private double[] getModuleAngles() {
    // Get the angles from the CANcoders
    return [0,0,0,0];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
