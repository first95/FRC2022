/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ManuallyControlClimber extends CommandBase {
  // Stores whether or not the deploy button was pressed during the last loop
  private boolean wasDeployedButtonPressed = false;
  private Climber m_Climber;

  public ManuallyControlClimber(Climber climber) {
    m_Climber = climber;
    addRequirements(climber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // If the deploy button was not pressed during the last loop and is pressed
    // during the current loop,
    // toggle deploy and then set wasDeployed to true
    if (!wasDeployedButtonPressed && RobotContainer.oi.getClimberDeployed()) {
      m_Climber.toggleClimberDeploy();
      System.out.println("climber deploy has been toggled");
      wasDeployedButtonPressed = true;
    } else {
      // Set wasDeployed to false
      wasDeployedButtonPressed = false;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
