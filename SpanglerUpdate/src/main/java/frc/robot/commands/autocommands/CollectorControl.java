// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class CollectorControl extends CommandBase {
  private double speed;
  private boolean toggleDeploy;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CollectorControl(double speed_p, boolean toggleDeploy_p) {
    speed = speed_p;
    toggleDeploy = toggleDeploy_p;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.oi.auto_collector_deploy = toggleDeploy;
    RobotContainer.oi.auto_collect_speed = speed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
