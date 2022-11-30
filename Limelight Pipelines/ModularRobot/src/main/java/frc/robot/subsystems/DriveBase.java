// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivebase;

public class DriveBase extends SubsystemBase {
  /** This is a subsystem for a tank drivebase.  It handles odometry (via the pose estimator class) 
   * and has methods for directly controlling the drivebase.  Positive power to the motors will move
   * the robot forward.*/

  private CANSparkMax leftPod, rightPod, l2, r2, l3, r3;
  private RelativeEncoder leftEncoder, rightEncoder;
  private Solenoid brakes;
  private DifferentialDriveOdometry odometry;
  private PigeonIMU.GeneralStatus status = new PigeonIMU.GeneralStatus();
  public PigeonIMU imu = new PigeonIMU(Drivebase.PIGEON_IMU_ID);

  public DriveBase() {
    // Creates the motor controller objects for the drive motors. leftPod and rightPod are
    // the leader motors on each side, and the l# and r# motors are all the followers.
    // Add as many followers as necessary.
    leftPod = new CANSparkMax(Drivebase.LEFT_LEAD, MotorType.kBrushless);
    rightPod = new CANSparkMax(Drivebase.RIGHT_LEAD, MotorType.kBrushless);
    l2 = new CANSparkMax(Drivebase.LEFT_F1, MotorType.kBrushless);
    r2 = new CANSparkMax(Drivebase.RIGHT_F1, MotorType.kBrushless);
    l3 = new CANSparkMax(Drivebase.LEFT_F2, MotorType.kBrushless);
    r3 = new CANSparkMax(Drivebase.RIGHT_F2, MotorType.kBrushless);

    // Sets the current limit for all drive motors
    leftPod.setSmartCurrentLimit(Drivebase.CurrentLimit);
    rightPod.setSmartCurrentLimit(Drivebase.CurrentLimit);
    l2.setSmartCurrentLimit(Drivebase.CurrentLimit);
    r2.setSmartCurrentLimit(Drivebase.CurrentLimit);
    l3.setSmartCurrentLimit(Drivebase.CurrentLimit);
    r3.setSmartCurrentLimit(Drivebase.CurrentLimit);
    // Make the current limits permanent in the motor controllers
    leftPod.burnFlash();
    l2.burnFlash();
    l3.burnFlash();
    rightPod.burnFlash();
    r2.burnFlash();
    r3.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
