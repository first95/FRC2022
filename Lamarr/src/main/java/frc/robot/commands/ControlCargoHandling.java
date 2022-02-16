// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CargoHandling;
import frc.robot.RobotContainer;
import frc.robot.Constants.CargoHandling.CargoColor;
import frc.robot.subsystems.CargoHandler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * ControlCargoHandling operates both the collecting and shooting of cargo in
 * both autonomous and teleop modes.
 * It has several states to operate with a few main goals:
 * 1. Collect cargo via ground pickup
 * 2. Idenitfy the ball color
 * 2a. If it is the right color prepare to shoot
 * 2b. If it is the wrong color eject it
 * 3. Shoot ball via PIDF either by auto commands or teleop.
 * 
 * Note that ControlCargoHandling IS ALWAYS RUNNING!
 * Autonoumous control is based off variables located in OI.java.
 */
public class ControlCargoHandling extends CommandBase {
  private CargoHandler cargoHandler;

  private enum State {
    IDLE, INDEX, SHOOTING, EJECT_A, EJECT_B
  }

  private State currentState;
  private CargoColor currentCargoColor;
  private boolean isIndexerLoaded, isShooterLoaded, shootingRequested, wasIndexerLoaded, wasShooterLoaded,
      wasCollectorToggled;

  // For FSM motor control
  private double indexerRunSpeed, collectorRunSpeed, shooterRunSpeed, requestedCollectorSpeed, targetShooterSpeed;

  // For shooter PIDF
  private double actual_speed, speedError, speedErrorPercent, speedIntegral, speedDerivative, lastSpeedErrorPercent,
      targetPower, correction, cappedCorrection, kp, ki, kd;

  // For indexer delay for shoooting
  private int spinupDelayCount;
  private boolean shooterSpunUp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ControlCargoHandling(CargoHandler cargoHandler) {
    this.cargoHandler = cargoHandler;
    addRequirements(cargoHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;
    wasIndexerLoaded = false;
    wasShooterLoaded = false;
    wasCollectorToggled = false;
    lastSpeedErrorPercent = 0;
    shooterSpunUp = false;
    spinupDelayCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Output shooter speeds
    SmartDashboard.putNumber("ProcessVariable", cargoHandler.getShooterSpeed());
    // Toggle pneumatics if needed
    if (RobotContainer.oi.getGroundPickUpDeployed() && !wasCollectorToggled) {
      wasCollectorToggled = true;
      cargoHandler.toggleCollectorDeploy();
    } else if (!RobotContainer.oi.getGroundPickUpDeployed() && wasCollectorToggled) {
      wasCollectorToggled = false;
    }

    // Manual Eject (add to statemachine if needed)
    /*if (RobotContainer.oi.getEjectButton()) {
      cargoHandler.runCollector(-0.5);
      cargoHandler.runIndexer(-0.5);
      return;
    } else {
      cargoHandler.runCollector(0);
      cargoHandler.runIndexer(0);
    }*/

    currentCargoColor = cargoHandler.getCargoColor();
    isIndexerLoaded = cargoHandler.getIndexerLoaded();
    isShooterLoaded = cargoHandler.getShooterLoaded();
    shootingRequested = RobotContainer.oi.getShooterButton();
    requestedCollectorSpeed = RobotContainer.oi.getGroundPickUpRollerAxis();

    if (RobotContainer.oi.auto_shooting) {
      targetShooterSpeed = RobotContainer.oi.auto_shooting_speed;
    } else {
      targetShooterSpeed = 4100;
    }

    // Determine where we are in the cargo lifecycle
    switch (currentState) {
      case IDLE:
        SmartDashboard.putString("State", "IDLE");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = 0;
        shooterRunSpeed = 0;

        if ((currentCargoColor == CargoColor.RIGHT) && !isShooterLoaded) {
          currentState = State.INDEX;
        }
        if ((currentCargoColor == CargoColor.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_A;
        }
        if ((currentCargoColor == CargoColor.WRONG) && isShooterLoaded) {
          currentState = State.EJECT_B;
        }
        if (shootingRequested) {
          currentState = State.SHOOTING;
        }
        break;
      case INDEX:
        SmartDashboard.putString("State", "INDEX");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = CargoHandling.INDEXING_SPEED;
        shooterRunSpeed = 0;

        if (isShooterLoaded || !isIndexerLoaded) {
          currentState = State.IDLE;
        }
        if ((currentCargoColor == CargoColor.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_A;
        }
        if ((currentCargoColor == CargoColor.WRONG) && isShooterLoaded) {
          currentState = State.EJECT_B;
        }
        if (shootingRequested) {
          currentState = State.SHOOTING;
        }
        break;
      case EJECT_A:
        SmartDashboard.putString("State", "EJECT_A");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = CargoHandling.INDEXING_SPEED;
        shooterRunSpeed = CargoHandling.SHOOTER_SLOW_SPEED;

        if (!isShooterLoaded && wasShooterLoaded) {
          currentState = State.IDLE;
        }
        break;
      case EJECT_B:
        SmartDashboard.putString("State", "EJECT_B");
        collectorRunSpeed = CargoHandling.COLLECTOR_REVERSE;
        indexerRunSpeed = CargoHandling.INDEXER_REVERSE;
        shooterRunSpeed = 0;

        if ((currentCargoColor == CargoColor.RIGHT) || (currentCargoColor == CargoColor.NONE)) {
          currentState = State.IDLE;
        }
      case SHOOTING:
        SmartDashboard.putString("State", "SHOOTING");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = 0;
        shooterRunSpeed = targetShooterSpeed;

        if (!shootingRequested) {
          currentState = State.IDLE;
        }
    }

    // Collect if scheduled to collect
    cargoHandler.runCollector(collectorRunSpeed);

    // Index id scheduled to index
    if (currentState != State.SHOOTING) {
      cargoHandler.runIndexer(indexerRunSpeed);
    }

    // Shoot if scheduled to shoot
    runShooterPIDF(shooterRunSpeed);

    wasIndexerLoaded = isIndexerLoaded;
    wasShooterLoaded = isShooterLoaded;

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

  private void runShooterPIDF(double targetRPM) {
    kp = CargoHandling.SHOOTER_KP;
    ki = CargoHandling.SHOOTER_KI;
    kd = CargoHandling.SHOOTER_KD;
    if (targetRPM != 0) {
      actual_speed = cargoHandler.getShooterSpeed();
      SmartDashboard.putNumber("ProcessVariable", actual_speed);
        
      targetPower = targetRPM * CargoHandling.RPM_TO_SHOOTER_POWER_CONVERSION;
        
      speedError = targetRPM - actual_speed;
      speedErrorPercent = speedError / targetRPM;

      if (Math.abs(targetRPM - actual_speed) <= 200) { // Anti-Windup
        speedIntegral += speedErrorPercent;
      }
      
      speedDerivative = speedErrorPercent - lastSpeedErrorPercent;
      
      correction = (kp * speedErrorPercent) +
      (ki * speedIntegral) +
      (kd * speedDerivative) +
      targetPower;
      cappedCorrection = Math.min(correction, 1.0);
      
      cargoHandler.runShooter(cappedCorrection);
      
      lastSpeedErrorPercent = speedErrorPercent;

      if (((targetRPM - actual_speed) <= CargoHandling.SHOOTER_SPEED_TOLERANCE) || shooterSpunUp) {
        spinupDelayCount++;
        shooterSpunUp = true;
        cargoHandler.runIndexer(CargoHandling.SHOOTING_INDEXER_SPEED);
      }

    } else {
      cargoHandler.runShooter(0);
      spinupDelayCount = 0;
      shooterSpunUp = false;
    }
  }
}
