// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ControlClimber extends CommandBase {
  private Climber climber;

  private boolean wasPneumaticsToggled;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ControlClimber(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.oi.getClimberUnspoolButton())
      climber.setSpeed(-0.8);
    else if (RobotContainer.oi.getClimberSpoolButton())
      climber.setSpeed(0.8);
    else
      climber.setSpeed(0);

    if (RobotContainer.oi.getClimberButton() && !wasPneumaticsToggled) {
      wasPneumaticsToggled = true;
      climber.togglePistons();
    } else if (!RobotContainer.oi.getClimberButton() && wasPneumaticsToggled) {
      wasPneumaticsToggled = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
