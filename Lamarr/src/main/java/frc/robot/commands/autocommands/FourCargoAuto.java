// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import frc.robot.Constants.CargoHandling;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;

/** An example command that uses an example subsystem. */
public class FourCargoAuto extends SequentialCommandGroup {
  public FourCargoAuto(DriveBase drivebase, LimeLight limelightport, Trajectory [] trajectories) {
    addRequirements(drivebase);
    addRequirements(limelightport);

    // Deploy and run the collector
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;
      RobotContainer.oi.auto_collect_speed = 0.8;}));
    // Drive to the first cargo
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_TWO_CARGO_GET_FIRST]));
    //addCommands(new WaitCommand(0.1));
    // Stop the collector
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;}));
    // Drive back to the hub
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_TWO_CARGO_SHOOT_FIRST]));
    // Pew Pew
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_shooting_speed = CargoHandling.SHOOTING_HIGH_SPEED;
      RobotContainer.oi.auto_roller_speed = CargoHandling.ROLLER_HIGH_SPEED;
      RobotContainer.oi.auto_shooting = true;}));
    addCommands(new WaitCommand(2));
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_shooting = false;
      RobotContainer.oi.auto_shooting_speed = 0;
      RobotContainer.oi.auto_roller_speed = 0;
      RobotContainer.oi.auto_collect_speed = 0;}));

    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_CARGO_LINEUP]));

    // Start the collector
    addCommands(new InstantCommand(() ->
    {RobotContainer.oi.auto_collector_toggle = true;
    RobotContainer.oi.auto_collect_speed = 0.8;}));
    // Drive through and to the next two cargo
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_CARGO_GET_TWO]));
    //addCommands(new WaitCommand(0.1));
    // Stop and retract the collector
    addCommands(new InstantCommand(() ->
      {RobotContainer.oi.auto_collector_toggle = true;}));
    // Drive back to the hub
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FOUR_CARGO_SHOOT_SECOND]));
    // Pew Pew
    addCommands(new AutoAim(true, drivebase, limelightport).withTimeout(2));
  }
}
