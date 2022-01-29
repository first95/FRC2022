// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;


import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveBase;

/** An example command that uses an example subsystem. */
public class AutoMoves extends SequentialCommandGroup {
  public AutoMoves(DriveBase drivebase, Trajectory [] trajectories) {
    addRequirements(drivebase);
    /*addCommands(new CollectorControl(1, true));
    addCommands(new WaitCommand(2));
    addCommands(new FollowTrajectory(drivebase, trajectories[0]));
    addCommands(new WaitCommand(0.5));
    addCommands(new CollectorControl(0, false));
    addCommands(new WaitCommand(0.5));
    addCommands(new FollowTrajectory(drivebase, trajectories[1]));
    addCommands(new WaitCommand(2));*/
    addCommands(
      new InstantCommand(() -> {drivebase.resetGyro();
        drivebase.resetOdometry(trajectories[0].getInitialPose());
        drivebase.driveWithTankControls(0.2, -0.2);})
      .andThen(new WaitCommand(4))
      .andThen(new InstantCommand(() -> {drivebase.driveWithTankControls(0, 0);
        SmartDashboard.putNumber("Range 1 Power", drivebase.getWheelPositions()[0]);
        SmartDashboard.putNumber("Range 2 Power", drivebase.getWheelPositions()[1]);})));
  }
}
