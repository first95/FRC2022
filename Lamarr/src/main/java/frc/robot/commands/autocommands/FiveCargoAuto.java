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
import frc.robot.commands.Shoot;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterHood;

/** An example command that uses an example subsystem. */
public class FiveCargoAuto extends SequentialCommandGroup {
  public FiveCargoAuto(DriveBase drivebase, LimeLight limelightport, ShooterHood shooterhood, Trajectory [] trajectories) {
    addRequirements(drivebase);
    addRequirements(limelightport);

    Trajectory firstTrajectory = trajectories[Auton.FiveBall1_Backup].concatenate(trajectories[Auton.FiveBall2_K1]
      .concatenate(trajectories[Auton.FiveBall3_Get3]));

    
    // Pew
    addCommands(new Shoot(true, drivebase, limelightport, shooterhood).withTimeout(1));
    // Deploy and run the collector
    addCommands(new InstantCommand(() -> 
      {RobotContainer.oi.auto_collector_toggle = true;
      RobotContainer.oi.auto_collect_speed = 0.8;}));
    // Drive to the first and second cargo
    addCommands(new FollowTrajectory(drivebase, firstTrajectory));
    // Retract the collector
    addCommands(new InstantCommand(() -> {RobotContainer.oi.auto_collector_toggle = true;}));
    // Pew Pew
    addCommands(new Shoot(true, drivebase, limelightport, shooterhood).withTimeout(2));
    // Deploy collector
    addCommands(new InstantCommand(() -> {RobotContainer.oi.auto_collector_toggle = true;}));
    // Drive to terminal
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FiveBall4_Get45]));
    // Wait for HP cargo load
    addCommands(new WaitCommand(1));
    // Drive closer to hub
    addCommands(new FollowTrajectory(drivebase, trajectories[Auton.FiveBall5_Shoot45]));
    // Retract the collector
    addCommands(new InstantCommand(() -> {RobotContainer.oi.auto_collector_toggle = true;}));
    // Pew Pew
    addCommands(new AutoAim(true, drivebase, limelightport, shooterhood).withTimeout(5));
  }
}
