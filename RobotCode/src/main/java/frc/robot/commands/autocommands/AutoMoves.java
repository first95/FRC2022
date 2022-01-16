/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveBase;

/**
 * A group of auto moves to run in sequence.
 */
public class AutoMoves extends SequentialCommandGroup {
  public AutoMoves(DriveBase drivebase) {
    addCommands(new FollowTrajectory(drivebase));
  }

}
