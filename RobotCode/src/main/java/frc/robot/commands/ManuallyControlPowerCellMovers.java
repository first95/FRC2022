/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PowerCellMover;

public class ManuallyControlPowerCellMovers extends CommandBase {

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.8;
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double current_speed = 0;

  public static double MANUAL_RUN_SPEED_INDEXER = 0.8;

  public static double spinningSpeed = 0.8;
  public static double intakeSpeed = 0.8;

  private boolean wasDeployedButtonPressed = false;

private PowerCellMover m_PowerCellMover;

  public ManuallyControlPowerCellMovers(PowerCellMover powerCellMover) {
    m_PowerCellMover = powerCellMover; 
    addRequirements(powerCellMover);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Shooter speed setting
    if(RobotContainer.oi.getShooterButton()) {
      m_PowerCellMover.runShooterOpen(MANUAL_RUN_SPEED_SHOOTER);
      current_speed = MANUAL_RUN_SPEED_SHOOTER;
    } else {
      // Slowing down motor and don't want to do it too fast
      if (current_speed < MIN_RUN_SPEED) {
        current_speed = 0;
      } else {
        current_speed = current_speed*MANUAL_RUN_SPEED_SHOOTER;
      }
      m_PowerCellMover.runShooterOpen(current_speed);
    }

    // Indexer speed setting
    if (RobotContainer.oi.getRunIndexer()) {
      m_PowerCellMover.runIndexer(MANUAL_RUN_SPEED_INDEXER);
    } else {
      m_PowerCellMover.runIndexer(0);
    }

    // Singulator speed setting
    if(RobotContainer.oi.getSingulatorButton()) {
      m_PowerCellMover.setSingulatorSpeed(spinningSpeed);
    } else {
      m_PowerCellMover.setSingulatorSpeed(0);
    }


    // Ground pickup deploy and speed settings
    // If the deploy button was not pressed during the last loop and is pressed during the current loop,
      // toggle deploy
    if (!wasDeployedButtonPressed && RobotContainer.oi.getGroundPickUpDeployed())
    {
      m_PowerCellMover.toggleGroundPickUpDeploy();
      // System.out.println("ground pickup has been deployed");
    }

    m_PowerCellMover.setRollerSpeed(RobotContainer.oi.getGroundPickUpRollerAxis());

    SmartDashboard.putBoolean("SingulatorOccupied",m_PowerCellMover.getSingulatorSensor());
    SmartDashboard.putBoolean("IndexerEntranceOccupied",m_PowerCellMover.getIndexerEntranceSensor());
    SmartDashboard.putBoolean("IndexerPosition1Occupied",m_PowerCellMover.getIndexerLoadedSensor());
    SmartDashboard.putBoolean("ShooterLoaded",m_PowerCellMover.getShooterLoadedSensor());

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
