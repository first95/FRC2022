// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CargoHandling;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CargoHandling.CargoColor;
import frc.robot.subsystems.CargoHandler;
import frc.robot.utils.Logger;
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
 * DO NOT take control of the cargo handler with another command.
 * Autonoumous control is based off variables located in OI.java.
 */
public class ControlCargoHandling extends CommandBase {
  private CargoHandler cargoHandler;

  private enum State {
    IDLE, PRESPOOL, INDEX, SHOOTING, EJECT_A, EJECT_B
  }

  // FSM variables
  private State currentState;
  private CargoColor currentCargoColor;
  private boolean isShooterLoaded, shootingRequested, wasShooterLoaded,
      wasCollectorToggled, haveTarget;
  private int ejectionTimer;

  // For FSM motor control
  private double indexerRunSpeed, collectorRunSpeed, shooterRunSpeed, requestedCollectorSpeed, targetShooterSpeed,
      rollerRunSpeed, targetRollerSpeed, floorDistance;

  // For shooter PF
  private double actual_speed, speedError, speedErrorPercent, targetPower, correction, cappedCorrection, kp,
      roller_speed, rollerSpeedError, rollerSpeedErrorPercent, rollerTargetPower, rollerCorrection,
      rollerCappedCorrection,
      rollerkP;

  public ControlCargoHandling(CargoHandler cargoHandler) {
    this.cargoHandler = cargoHandler;
    addRequirements(cargoHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;
    wasShooterLoaded = false;
    wasCollectorToggled = false;
    ejectionTimer = 0;

    SmartDashboard.putNumber("Shooter Slope", CargoHandling.FAR_SHOOTER_SPEED_M);
    SmartDashboard.putNumber("Shooter Intercept", CargoHandling.FAR_SHOOTER_SPEED_B);
    SmartDashboard.putNumber("Shooter Ratio", CargoHandling.SHOOTER_RATIO);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Output shooter speed
    // SmartDashboard.putNumber("ProcessVariable", cargoHandler.getShooterSpeed());

    // Toggle pneumatics if needed
    if (RobotContainer.oi.getGroundPickUpDeployed() && !wasCollectorToggled) {
      wasCollectorToggled = true;
      cargoHandler.toggleCollectorDeploy();
    } else if (!RobotContainer.oi.getGroundPickUpDeployed() && wasCollectorToggled) {
      wasCollectorToggled = false;
    }

    currentCargoColor = cargoHandler.getCargoColor();
    isShooterLoaded = cargoHandler.getShooterLoaded();
    haveTarget = (SmartDashboard.getNumber("port-Target Valid?", 0) == 1) ? true : false;
    shootingRequested = RobotContainer.oi.getShooterButton();
    requestedCollectorSpeed = RobotContainer.oi.getGroundPickUpRollerAxis();
    floorDistance = SmartDashboard.getNumber("port-Horiz. Range", 0);

    if (RobotContainer.oi.auto_shooting) {
      targetShooterSpeed = RobotContainer.oi.auto_shooting_speed;
      targetRollerSpeed = RobotContainer.oi.auto_roller_speed;
    } else if (RobotContainer.oi.manual_shooting_high) {
      targetShooterSpeed = CargoHandling.MANUAL_SHOOTING_SPEED;
      targetRollerSpeed = CargoHandling.MANUAL_SHOOTING_SPEED * SmartDashboard.getNumber("Shooter Ratio", CargoHandling.SHOOTER_RATIO);
    } else {
      targetShooterSpeed = CargoHandling.SHOOTING_LOW_SPEED;
      targetRollerSpeed = CargoHandling.ROLLER_LOW_SPEED;
    }

    // Determine where we are in the cargo lifecycle
    switch (currentState) {
      case IDLE:
        SmartDashboard.putString("State", "IDLE");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = 0;
        shooterRunSpeed = CargoHandling.SHOOTER_IDLE_SPEED;
        rollerRunSpeed = CargoHandling.ROLLER_IDLE_SPEED;

        if (haveTarget && isShooterLoaded) {
          currentState = State.PRESPOOL;
          Logger.info("to PRESPOOL from IDLE");
        }
        if ((currentCargoColor == CargoColor.RIGHT) && !isShooterLoaded) {
          currentState = State.INDEX;
          Logger.info("to INDEX from IDLE");
        }
        if ((currentCargoColor == CargoColor.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_A;
          Logger.info("to EJECT_A from IDLE");
        }
        if (((currentCargoColor == CargoColor.WRONG) && isShooterLoaded) || RobotContainer.oi.getEjectButton()) {
          currentState = State.EJECT_B;
          Logger.info("to EJECT_B from IDLE");
        }
        if (shootingRequested) {
          currentState = State.SHOOTING;
          Logger.info("to SHOOTING fom IDLE");
          shooterRunSpeed = RobotContainer.oi.auto_shooting_speed;
          rollerRunSpeed = RobotContainer.oi.auto_roller_speed;
        }
        break;
      case PRESPOOL:
      SmartDashboard.putString("State", "PRESPOOL");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = 0;
        shooterRunSpeed = (floorDistance <= Constants.Vision.BREAKPOINT) ?
          CargoHandler.distanceToShooterRPM(floorDistance) :
          CargoHandler.farDistanceToShooterRPM(floorDistance);
        rollerRunSpeed = shooterRunSpeed * SmartDashboard.getNumber("Shooter Ratio", Constants.CargoHandling.SHOOTER_RATIO);

        if (shootingRequested) {
          currentState = State.SHOOTING;
          Logger.info("to SHOOTING from PRESPOOL");
          shooterRunSpeed = RobotContainer.oi.auto_shooting_speed;
          rollerRunSpeed = RobotContainer.oi.auto_roller_speed;
        }
        if (((currentCargoColor == CargoColor.WRONG) && isShooterLoaded) || RobotContainer.oi.getEjectButton()) {
          currentState = State.EJECT_B;
          Logger.info("to EJECT_B from PRESPOOL");
        }
        break;

      case INDEX:
        SmartDashboard.putString("State", "INDEX");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = CargoHandling.INDEXING_SPEED;
        shooterRunSpeed = CargoHandling.SHOOTER_IDLE_SPEED;
        rollerRunSpeed = CargoHandling.ROLLER_IDLE_SPEED;

        if (isShooterLoaded) {
          currentState = State.IDLE;
        }
        if ((currentCargoColor == CargoColor.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_A;
          Logger.info("to EJECT_A from INDEX");
        }
        if ((currentCargoColor == CargoColor.WRONG) && isShooterLoaded) {
          currentState = State.EJECT_B;
          Logger.info("to EJECT_B from INDEX");
        }
        if (shootingRequested) {
          currentState = State.SHOOTING;
          Logger.info("to SHOOTING from INDEX");
          shooterRunSpeed = RobotContainer.oi.auto_shooting_speed;
          rollerRunSpeed = RobotContainer.oi.auto_roller_speed;
        }
        if (haveTarget && isShooterLoaded) {
          currentState = State.PRESPOOL;
          Logger.info("to PRESPOOL from INDEX");
        }
        break;
      case EJECT_A:
        SmartDashboard.putString("State", "EJECT_A");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = CargoHandling.INDEXING_SPEED;
        shooterRunSpeed = CargoHandling.SHOOTER_SLOW_SPEED;
        rollerRunSpeed = CargoHandling.ROLLER_EJECT_SPEED;

        if (!isShooterLoaded && wasShooterLoaded) {
          currentState = State.IDLE;
          Logger.info("to IDLE from EJECT_A");
        }
        break;
      case EJECT_B:
        SmartDashboard.putString("State", "EJECT_B");
        collectorRunSpeed = CargoHandling.COLLECTOR_REVERSE;
        indexerRunSpeed = CargoHandling.INDEXER_REVERSE;
        shooterRunSpeed = CargoHandling.SHOOTER_IDLE_SPEED;
        rollerRunSpeed = CargoHandling.ROLLER_IDLE_SPEED;

        if ((currentCargoColor == CargoColor.RIGHT) || (currentCargoColor == CargoColor.NONE) || (ejectionTimer > 0)) {
          ejectionTimer++;
        }

        if (ejectionTimer >= 30) {
          ejectionTimer = 0;
          currentState = State.IDLE;
          Logger.info("to IDLE from EJECT_B");
        }
        break;
      case SHOOTING:
        SmartDashboard.putString("State", "SHOOTING");
        collectorRunSpeed = requestedCollectorSpeed;
        indexerRunSpeed = CargoHandling.SHOOTING_INDEXER_SPEED;
        shooterRunSpeed = targetShooterSpeed;
        rollerRunSpeed = targetRollerSpeed;

        if (!shootingRequested) {
          currentState = State.IDLE;
          Logger.info("to IDLE from SHOOTING");
        }
        break;
    }

    // Collect if scheduled to collect
    cargoHandler.runCollector(collectorRunSpeed);

    // Unless the shooter is to be run, run the indexer at the commanded speed
    // If the shooter is running, the indexer will be handled later.
    if (currentState != State.SHOOTING) {
      cargoHandler.runIndexer(indexerRunSpeed);
      Logger.info("Setting indexer to " + String.valueOf(indexerRunSpeed));
    }

    // Shoot if scheduled to shoot
    runShooterPF(shooterRunSpeed, rollerRunSpeed);

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

  private void runShooterPF(double targetRPM, double rollerRPM) {
    kp = CargoHandling.SHOOTER_KP;
    rollerkP = SmartDashboard.getNumber("kp", CargoHandling.ROLLER_KP);
    if ((targetRPM != 0) || (rollerRPM != 0)) {
      actual_speed = cargoHandler.getShooterSpeed();
      roller_speed = cargoHandler.getRollerSpeed();
      SmartDashboard.putNumber("ProcessVariable", actual_speed);
      SmartDashboard.putNumber("Roller Speed", roller_speed);

      targetPower = targetRPM * CargoHandling.RPM_TO_SHOOTER_POWER_CONVERSION;
      rollerTargetPower = rollerRPM * CargoHandling.RPM_TO_ROLLER_POWER_CONVERSION;

      speedError = targetRPM - actual_speed;
      speedErrorPercent = speedError / targetRPM;
      rollerSpeedError = rollerRPM - roller_speed;
      rollerSpeedErrorPercent = rollerSpeedError / rollerRPM;

      correction = (kp * speedErrorPercent) + targetPower;
      cappedCorrection = Math.min(correction, 1.0);
      rollerCorrection = (rollerkP * rollerSpeedErrorPercent) + rollerTargetPower;
      rollerCappedCorrection = Math.min(rollerCorrection, 1.0);

      cargoHandler.runShooter(cappedCorrection);
      cargoHandler.runRoller(rollerCappedCorrection);

      if (currentState == State.SHOOTING) {
        if (((targetRPM - actual_speed) <= CargoHandling.SHOOTER_SPEED_TOLERANCE) &&
            (((rollerRPM - roller_speed) <= CargoHandling.ROLLER_SPEED_TOLERANCE))) {
          cargoHandler.runIndexer(indexerRunSpeed);
          Logger.info("Shooter spun up. Setting indexer to " + String.valueOf(indexerRunSpeed));
        } else {
          cargoHandler.runIndexer(0);
          Logger.info("Shooter too slow. Stopping indexer");
        }
      }

    } else {
      cargoHandler.runShooter(0);
      cargoHandler.runRoller(0);
    }
  }
}
