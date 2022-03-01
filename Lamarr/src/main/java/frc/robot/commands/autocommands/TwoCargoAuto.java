// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

/** An example command that uses an example subsystem. */
public class TwoCargoAuto extends SequentialCommandGroup {
  public TwoCargoAuto(DriveBase drivebase, LimeLight limelightport, Trajectory [] trajectories) {
    addRequirements(drivebase);
    addRequirements(limelightport);

    // Deploy and run the collector
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;
      RobotContainer.oi.auto_collect_speed = 0.6;}));
    // Drive to the first cargo
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_TWO_CARGO_GET_FIRST]));
    // Stop and retract the collector
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collect_speed = 0;
      RobotContainer.oi.auto_collector_toggle = true;}));
    // Drive back to the hub
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_TWO_CARGO_SHOOT_FIRST]));
    // Pew Pew
    addCommands(new AutoAim(true, drivebase, limelightport).withTimeout(2));
  }
}