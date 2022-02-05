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
 * AutoCargoHandling operates both the collecting and shooting of cargo in
 * both autonomous and teleop modes.
 * It has several states to operate with a few main goals:
 * 1. Collect cargo via ground pickup
 * 2. Idenitfy the ball color
 * 2a. If it is the right color prepare to shoot
 * 2b. If it is the wrong color eject it
 * 3. Shoot ball via PIDF either by auto commands or teleop.
 * 
 * Note that AutoCargoHandling IS ALWAYS RUNNING!
 * Autonoumous control is based off variables located in OI.java.
 */
public class AutoCargoHandling extends CommandBase {
  private CargoHandler cargoHandler;

  private enum State {
    IDLE, SHOOTING, INDEX, EJECT_A, EJECT_B
  }

  private State currentState;
  private CargoColor currentCargoColor;
  private boolean isIndexerLoaded, isShooterLoaded, shootingRequested, wasIndexerLoaded, wasShooterLoaded;

  // For FSM motor control
  private double indexerRunSpeed, collectorRunSpeed, shooterRunSpeed, requestedCollectorSpeed, targetShooterSpeed;

  // For shooter PIDF
  private double actual_speed, speedError, speedErrorPercent, speedIntegral, speedDerivative, lastSpeedErrorPercent,
      targetPower, correction, cappedCorrection;

  // For indexer delay for shoooting
  private int spinupDelayCount;
  private boolean shooterSpunUp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoCargoHandling(CargoHandler cargoHandler) {
    this.cargoHandler = cargoHandler;
    addRequirements(cargoHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;
    wasIndexerLoaded = false;
    lastSpeedErrorPercent = 0;
    shooterSpunUp = false;
    spinupDelayCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentCargoColor = cargoHandler.getCargoColor();
    isIndexerLoaded = cargoHandler.getIndexerLoaded();
    isShooterLoaded = cargoHandler.getShooterLoaded();
    shootingRequested = RobotContainer.oi.getShooterButton();
    requestedCollectorSpeed = RobotContainer.oi.getGroundPickUpRollerAxis();

    if (RobotContainer.oi.auto_shooting) {
      targetShooterSpeed = RobotContainer.oi.auto_shooting_speed;
    } else {
      targetShooterSpeed = 2000; // Placeholder RPM
    }

    switch (currentState) {
      case IDLE:
        indexerRunSpeed = 0;
        collectorRunSpeed = requestedCollectorSpeed;
        shooterRunSpeed = 0;

        if ((currentCargoColor == CargoColor.RIGHT) && (isShooterLoaded == false)) {
          currentState = State.INDEX;
        }
        if (shootingRequested) {
          currentState = State.SHOOTING;
        }
        if ((currentCargoColor == CargoColor.WRONG) && (isShooterLoaded == false)) {
          currentState = State.EJECT_A;
        }
        if ((currentCargoColor == CargoColor.WRONG) && (isShooterLoaded == true)) {
          currentState = State.EJECT_B;
        }
        break;
      case INDEX:
        indexerRunSpeed = CargoHandling.INDEXING_SPEED;
        collectorRunSpeed = requestedCollectorSpeed;
        shooterRunSpeed = 0;

        if (((isIndexerLoaded == false) && (wasIndexerLoaded == true)) || isShooterLoaded == true) {
          currentState = State.IDLE;
        }
        if (shootingRequested) {
          currentState = State.SHOOTING;
        }
        if ((currentCargoColor == CargoColor.WRONG) && (isShooterLoaded == false)) {
          currentState = State.EJECT_A;
        }
        if ((currentCargoColor == CargoColor.WRONG) && (isShooterLoaded == true)) {
          currentState = State.EJECT_B;
        }
        break;
      case SHOOTING:
        indexerRunSpeed = 0; // Indexer will be set by shooter PID after spinup
        collectorRunSpeed = requestedCollectorSpeed;
        shooterRunSpeed = targetShooterSpeed;

        if (shootingRequested == false) {
          currentState = State.IDLE;
        }
        break;
      case EJECT_A:
        indexerRunSpeed = CargoHandling.INDEXING_SPEED;
        collectorRunSpeed = requestedCollectorSpeed;
        shooterRunSpeed = CargoHandling.SHOOTER_SLOW_SPEED;

        if ((isShooterLoaded == false) && (wasShooterLoaded == true)) {
          currentState = State.IDLE;
        }
        break;
      case EJECT_B:
        indexerRunSpeed = CargoHandling.INDEXER_REVERSE;
        collectorRunSpeed = CargoHandling.COLLECTOR_REVERSE;
        shooterRunSpeed = 0;

        if (currentCargoColor == CargoColor.NONE) {
          currentState = State.IDLE;
        }
        break;
    }
    cargoHandler.runCollector(collectorRunSpeed);
    if (currentState != State.SHOOTING) {
      cargoHandler.runIndexer(indexerRunSpeed);
    }
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

      correction = (CargoHandling.SHOOTER_KP * speedErrorPercent) +
          (CargoHandling.SHOOTER_KI * speedIntegral) +
          (CargoHandling.SHOOTER_KD * speedDerivative) +
          targetPower;
      cappedCorrection = Math.min(correction, 1.0);

      cargoHandler.runShooter(cappedCorrection);

      lastSpeedErrorPercent = speedErrorPercent;

      if ((Math.abs(targetRPM - actual_speed) <= CargoHandling.SHOOTER_SPEED_TOLERANCE) || shooterSpunUp) {
        spinupDelayCount++;
        shooterSpunUp = true;
      }

      if (spinupDelayCount >= 10) {
        cargoHandler.runIndexer(CargoHandling.SHOOTING_INDEXER_SPEED);
      } else {
        cargoHandler.runIndexer(0);
      }
    } else {
      cargoHandler.runShooter(0);
      cargoHandler.runIndexer(0);
    }
  }
}
