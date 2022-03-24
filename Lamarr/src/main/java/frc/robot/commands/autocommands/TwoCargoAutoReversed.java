// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

/** An example command that uses an example subsystem. */
public class TwoCargoAutoReversed extends SequentialCommandGroup {
  public TwoCargoAutoReversed(DriveBase drivebase, LimeLight limelightport, Trajectory [] trajectories) {
    TrajectoryConfig config = new TrajectoryConfig(Drivebase.MAX_SPEED_MPS, Drivebase.MAX_ACCELERATION_MPSPS);
        config.setKinematics(Drivebase.DRIVE_KINEMATICS);
        config.setReversed(true);
        Trajectory back = new Trajectory();
        back = TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
            List.of(),
            new Pose2d(new Translation2d(-1, 0), new Rotation2d(0)),
             config);


    addRequirements(drivebase);
    addRequirements(limelightport);

    // Deploy and run the collector
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;
      RobotContainer.oi.auto_collect_speed = 0.6;}));
    // Drive to the first cargo
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.TWO_CARGO_REVERSED_GET]));
    addCommands(new WaitCommand(0.3));
    // Retract the collector
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collector_toggle = true;}));
    // Pew Pew
    addCommands(new AutoAim(true, drivebase, limelightport).withTimeout(5));
    // Ensure we leave the tarmac
    addCommands(new FollowTrajectory(drivebase, back));
  }
}
