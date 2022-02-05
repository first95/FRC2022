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
public class ManuallyRunCollector extends CommandBase {
    private CargoHandler cargoHandler;

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
    public ManuallyRunCollector(CargoHandler cargoHandler) {
        this.cargoHandler = cargoHandler;
        addRequirements(cargoHandler);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Collect if scheduled to collect
        cargoHandler.runCollector(RobotContainer.oi.getGroundPickUpRollerAxis());
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
