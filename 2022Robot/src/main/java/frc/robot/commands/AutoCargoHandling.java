// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CargoColor;
import frc.robot.subsystems.CargoHandler;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoCargoHandling extends CommandBase {
  private CargoHandler cargoHandler;

  private enum State {
    IDLE, SHOOTING, INDEX, EJECT_A, EJECT_B
  }

  private State currentState;
  private CargoColor currentCargoColor;
  private boolean isIndexerLoaded, isShooterLoaded, shootingRequested, wasIndexerLoaded, wasShooterLoaded;

  private double indexerRunSpeed, collectorRunSpeed, shooterRunSpeed, requestedCollectorSpeed, targetShooterSpeed;


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
      targetShooterSpeed = 2000;  // Placeholder RPM
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
        indexerRunSpeed = Constants.INDEXING_SPEED;
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
        indexerRunSpeed = Constants.INDEXING_SPEED;
        collectorRunSpeed = requestedCollectorSpeed;
        shooterRunSpeed = Constants.SHOOTER_SLOW_SPEED;

        if ((isShooterLoaded == false) && (wasShooterLoaded == true)) {
          currentState = State.IDLE;
        }
        break;
      case EJECT_B:
        indexerRunSpeed = Constants.INDEXER_REVERSE;
        collectorRunSpeed = Constants.COLLECTOR_REVERSE;
        shooterRunSpeed = 0;

        if (currentCargoColor == CargoColor.NONE) {
          currentState = State.IDLE;
        }
        break;
    }
    cargoHandler.runCollector(collectorRunSpeed);
    cargoHandler.runIndexer(indexerRunSpeed);
    cargoHandler.runShooter(shooterRunSpeed);

    wasIndexerLoaded = isIndexerLoaded;
    wasShooterLoaded = isShooterLoaded;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void runShooterPIDF(double targetRPM) {
    if (targetRPM != 0) {

    }
  }
}
