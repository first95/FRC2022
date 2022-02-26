// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

/** An example command that uses an example subsystem. */
public class AutoMoves extends SequentialCommandGroup {
  public AutoMoves(DriveBase drivebase, LimeLight limelightport, Trajectory [] trajectories) {
    addRequirements(drivebase);
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;
      RobotContainer.oi.auto_collect_speed = 0.6;}));
    addCommands(new FollowTrajectory(drivebase, trajectories[0]));
    addCommands(new WaitCommand(0.5));
    addCommands(new InstantCommand(() -> 
      RobotContainer.oi.auto_collect_speed = 0));
    addCommands(new FollowTrajectory(drivebase, trajectories[1]));
    addCommands(new AutoAim(drivebase, limelightport).withTimeout(2.5));
    addCommands(new InstantCommand(() ->
    {RobotContainer.oi.auto_collect_speed = 0.6;}));
    addCommands(new FollowTrajectory(drivebase, trajectories[2]));
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collect_speed = 0;
      RobotContainer.oi.auto_collector_toggle = true;}));
    addCommands(new FollowTrajectory(drivebase, trajectories[3]));
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_shooting_speed = Constants.CargoHandling.SHOOTING_SPEED;
      RobotContainer.oi.auto_shooting = true;}));
    addCommands(new WaitCommand(3));
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_shooting = false;
      RobotContainer.oi.auto_shooting_speed = 0;}));
  }
}
