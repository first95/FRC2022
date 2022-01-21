// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase;

/** An example command that uses an example subsystem. */
public class AutoMoves extends SequentialCommandGroup {
  public AutoMoves(DriveBase drivebase) {
    addCommands(new FollowTrajectory(drivebase));
  }
}
