// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.DriveBase;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class PathTest extends SequentialCommandGroup {
  private double maxSpeed, maxAccel;

  public PathTest(DriveBase drivebase) {
    addRequirements(drivebase);

    maxSpeed = SmartDashboard.getNumber("maxSpeed", 1);
    maxAccel = SmartDashboard.getNumber("maxAccel", 1);

    TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
    config.setKinematics(Drivebase.DRIVE_KINEMATICS);
    Trajectory forward = TrajectoryGenerator.generateTrajectory(
      new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 
      List.of(), 
      new Pose2d(new Translation2d(3, 0), new Rotation2d(0)),
      config);
    config.setReversed(true);
    Trajectory reverse = TrajectoryGenerator.generateTrajectory(
      new Pose2d(new Translation2d(3, 0), new Rotation2d(0)), 
      List.of(), 
      new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
      config);

    RamseteCommand forwardFollow = new RamseteCommand(
      forward,
      drivebase::getPose,
      new RamseteController(Drivebase.RAMSETE_B, Drivebase.RAMSETE_ZETA),
      new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA),
      Drivebase.DRIVE_KINEMATICS,
      drivebase::getWheelSpeeds,
      new PIDController(Drivebase.KP, 0, 0),
      new PIDController(Drivebase.KP, 0, 0),
      drivebase::tankDriveVolts,
      drivebase);

    RamseteCommand reverseFollow = new RamseteCommand(
      reverse,
      drivebase::getPose,
      new RamseteController(Drivebase.RAMSETE_B, Drivebase.RAMSETE_ZETA),
      new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA),
      Drivebase.DRIVE_KINEMATICS,
      drivebase::getWheelSpeeds,
      new PIDController(Drivebase.KP, 0, 0),
      new PIDController(Drivebase.KP, 0, 0),
      drivebase::tankDriveVolts,
      drivebase);
      
      addCommands(new InstantCommand(() -> {drivebase.resetOdometry(forward.getInitialPose());})
      .andThen(forwardFollow.andThen(() -> drivebase.tankDriveVolts(0, 0))));
      addCommands(new InstantCommand(() -> {drivebase.resetOdometry(reverse.getInitialPose());})
            .andThen(reverseFollow.andThen(() -> drivebase.tankDriveVolts(0, 0))));
  }
}
