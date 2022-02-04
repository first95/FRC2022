// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivebase;
import frc.robot.RobotContainer;

public class DriveBase extends SubsystemBase {

  private CANSparkMax leftPod, rightPod, l2, r2, l3, r3;
  private RelativeEncoder leftEncoder, rightEncoder;
  private DifferentialDriveOdometry odometry;
  private PigeonIMU.GeneralStatus status = new PigeonIMU.GeneralStatus();
  private PigeonIMU imu = new PigeonIMU(Drivebase.PIGEON_IMU_ID);


  public DriveBase() {
    leftPod = new CANSparkMax(Drivebase.LEFT_LEAD, MotorType.kBrushless);
    rightPod = new CANSparkMax(Drivebase.RIGHT_LEAD, MotorType.kBrushless);
    l2 = new CANSparkMax(Drivebase.LEFT_F1, MotorType.kBrushless);
    r2 = new CANSparkMax(Drivebase.RIGHT_F1, MotorType.kBrushless);
    l3 = new CANSparkMax(Drivebase.LEFT_F2, MotorType.kBrushless);
    r3 = new CANSparkMax(Drivebase.RIGHT_F2, MotorType.kBrushless);
    leftEncoder = leftPod.getEncoder();
    rightEncoder = rightPod.getEncoder();
    odometry = new DifferentialDriveOdometry(getYaw());

    l2.follow(leftPod);
    r2.follow(rightPod);
    l3.follow(leftPod);
    r3.follow(rightPod);

    rightPod.setInverted(true);
    leftEncoder.setPositionConversionFactor(Drivebase.METERS_PER_ROTATION);
    rightEncoder.setPositionConversionFactor(Drivebase.METERS_PER_ROTATION);
    leftEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_ROTATION / 60);
    rightEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_ROTATION / 60);
  }

  public void driveWithJoysticks() {
    double x = RobotContainer.oi.getForwardAxis();
    double y = RobotContainer.oi.getTurnAxis();
    x = Math.pow(x, 3);
    y = Math.pow(y, 3);
    leftPod.set(x - y);
    rightPod.set(x + y);
  }


  public void driveWithTankControls(double left, double right) {
    leftPod.set(left);
    rightPod.set(right);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftPod.setVoltage(-leftVolts);
    rightPod.setVoltage(-rightVolts);
  }

  /**
   * Returns a Rotation2d object of the robot's rotation
   */
  public Rotation2d getYaw() {
    return new Rotation2d(Math.toRadians(imu.getYaw()));
  }

  public void resetGyro() {
    imu.setYaw(0);
  }
  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double [] getWheelPositions() {
    double [] wheels = new double [2];
    wheels[0] = leftEncoder.getPosition();
    wheels[1] = rightEncoder.getPosition();
    return wheels;
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    odometry.resetPosition(pose, getYaw());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-leftEncoder.getVelocity(), -rightEncoder.getVelocity());
  }

  /**
   * Toggle motor brakes
   */
  public void setBreaks(boolean enabled) {
    leftPod.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    rightPod.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }
  
  @Override
  public void periodic() {
    imu.getGeneralStatus(status);
    odometry.update(getYaw(), -leftEncoder.getPosition(), -rightEncoder.getPosition());

    SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
    SmartDashboard.putNumber("X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("LeftM", -leftEncoder.getPosition());
    SmartDashboard.putNumber("RightM", -rightEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
