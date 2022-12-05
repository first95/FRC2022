// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivebase;

public class swerveBase extends SubsystemBase {
  
  private SwerveDriveKinematics kinematics;

  private ChassisSpeeds robotVelocity;

  private SwerveModuleState frontLeft, frontRight, backLeft, backRight;
  private SwerveModuleState[] moduleStates;

  private CANSparkMax driveFL, driveFR, driveBL, driveBR;
  private CANSparkMax steerFL, steerFR, steerBL, steerBR;

  private RelativeEncoder driveFLEncoder, driveFREncoder, driveBLEncoder, driveBREncoder;
  private RelativeEncoder steerFLEncoder, steerFREncoder, steerBLEncoder, steerBREncoder;

  private CANCoder absoluteEncoderFL, absoluteEncoderFR, absoluteEncoderBL, absoluteEncoderBR;

  private SparkMaxPIDController steerControllerFL, steerControllerFR, steerControllerBL, steerControllerBR;

  /** Creates a new swerve drivebase subsystem.  This will handle kinematics and
   * odometry. This also handles individual module control; it will use ChassisSpeeds objects
   * given by commands to constantly update wheel position and speed.*/
  public swerveBase() {
    // Swerve base kinematics object
    kinematics = new SwerveDriveKinematics(
      new Translation2d(Drivebase.FRONT_LEFT_X, Constants.Drivebase.FRONT_LEFT_Y),
      new Translation2d(Drivebase.FRONT_RIGHT_X, Constants.Drivebase.FRONT_RIGHT_Y),
      new Translation2d(Drivebase.BACK_LEFT_X, Constants.Drivebase.BACK_LEFT_Y),
      new Translation2d(Drivebase.BACK_RIGHT_X, Constants.Drivebase.BACK_RIGHT_Y)
    );

    // Current desired robot speed (initialized at 0, 0, 0)
    robotVelocity = new ChassisSpeeds();

    // Drive motors
    driveFL = new CANSparkMax(Drivebase.DRIVE_FL, MotorType.kBrushless);
    driveFR = new CANSparkMax(Drivebase.DRIVE_FR, MotorType.kBrushless);
    driveBL = new CANSparkMax(Drivebase.DRIVE_BL, MotorType.kBrushless);
    driveBR = new CANSparkMax(Drivebase.DRIVE_BR, MotorType.kBrushless);
    // Steering motors
    steerFL = new CANSparkMax(Drivebase.STEER_FL, MotorType.kBrushless);
    steerFR = new CANSparkMax(Drivebase.STEER_FR, MotorType.kBrushless);
    steerBL = new CANSparkMax(Drivebase.STEER_BL, MotorType.kBrushless);
    steerBR = new CANSparkMax(Drivebase.STEER_BR, MotorType.kBrushless);
    // Drive encoders (for speeds)
    driveFLEncoder = driveFL.getEncoder();
    driveFREncoder = driveFR.getEncoder();
    driveBLEncoder = driveBL.getEncoder();
    driveBREncoder = driveBR.getEncoder();
    // Steering relative encoders (less latency than CANCoder)
    steerFLEncoder = steerFL.getEncoder();
    steerFREncoder = steerFR.getEncoder();
    steerBLEncoder = steerBL.getEncoder();
    steerBREncoder = steerBR.getEncoder();
    // CANCoders, for azimuth on startup
    absoluteEncoderFL = new CANCoder(Drivebase.CANCODER_FL);
    absoluteEncoderFR = new CANCoder(Drivebase.CANCODER_FR);
    absoluteEncoderBL = new CANCoder(Drivebase.CANCODER_BL);
    absoluteEncoderBR = new CANCoder(Drivebase.CANCODER_BR);

    // Set position and velocity conversion factors for motor encoders.
    // The * 60 is because native units are RPM for some reason.
    driveFLEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
    driveFREncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
    driveBLEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
    driveBREncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);

    steerFLEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
    steerFREncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
    steerBLEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
    steerBREncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);

    driveFLEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION * 60);
    driveFREncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION * 60); 
    driveBLEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION * 60);
    driveBREncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION * 60);

    steerFLEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION * 60);
    steerFREncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION * 60);
    steerBLEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION * 60);
    steerBREncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION * 60);

    // Set the relative encoders to report the actual azimuth of the modules, as determined by
    // the CANcoders.  NOTE: this may need adjustment to make zero-positions and directions correct.
    steerFLEncoder.setPosition(absoluteEncoderFL.getAbsolutePosition());
    steerFREncoder.setPosition(absoluteEncoderFR.getAbsolutePosition());
    steerBLEncoder.setPosition(absoluteEncoderBL.getAbsolutePosition());
    steerBREncoder.setPosition(absoluteEncoderBR.getAbsolutePosition());

    // Set PIDF controller gains
    steerControllerFL.setP(Drivebase.MODULE_KP);
    steerControllerFR.setP(Drivebase.MODULE_KP);
    steerControllerBL.setP(Drivebase.MODULE_KP);
    steerControllerBR.setP(Drivebase.MODULE_KP);

    steerControllerFL.setI(Drivebase.MODULE_KI);
    steerControllerFR.setI(Drivebase.MODULE_KI);
    steerControllerBL.setI(Drivebase.MODULE_KI);
    steerControllerBR.setI(Drivebase.MODULE_KI);

    steerControllerFL.setD(Drivebase.MODULE_KD);
    steerControllerFR.setD(Drivebase.MODULE_KD);
    steerControllerBL.setD(Drivebase.MODULE_KD);
    steerControllerBR.setD(Drivebase.MODULE_KD);

    steerControllerFL.setFF(Drivebase.MODULE_KF);
    steerControllerFR.setFF(Drivebase.MODULE_KF);
    steerControllerBL.setFF(Drivebase.MODULE_KF);
    steerControllerBR.setFF(Drivebase.MODULE_KF);

    steerControllerFL.setIZone(Drivebase.MODULE_IZ);
    steerControllerFR.setIZone(Drivebase.MODULE_IZ);
    steerControllerBL.setIZone(Drivebase.MODULE_IZ);
    steerControllerBR.setIZone(Drivebase.MODULE_IZ);
  }

  @Override
  public void periodic() {
    // This handles drivebase control operations.  Order of operations is as follows:
    // 1. Convert the commanded ChassisSpeeds to SwerveModuleStates via kinematics
    // 2. Optimize the states
    // 3. Pass the optimized states to the azimuth and velocity controllers.

     moduleStates = kinematics.toSwerveModuleStates(robotVelocity);

     frontLeft = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(steerFLEncoder.getPosition()));
     frontRight = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(steerFREncoder.getPosition()));
     backLeft = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(steerBLEncoder.getPosition()));
     backRight = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(steerBREncoder.getPosition()));

     steerControllerFL.setReference(frontLeft.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
     steerControllerFR.setReference(frontRight.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
     steerControllerBL.setReference(backLeft.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
     steerControllerBR.setReference(backLeft.angle.getDegrees(), CANSparkMax.ControlType.kPosition);

     //Remember to add velocity control here too.
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
