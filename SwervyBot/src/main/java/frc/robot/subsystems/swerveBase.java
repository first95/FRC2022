// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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
  private SparkMaxPIDController driveControllerFL, driveControllerFR, driveControllerBL, driveControllerBR;

  private SwerveDriveOdometry odometry;
  private Field2d field = new Field2d();

  private double angle, lasttime;

  private Timer timer;

  /** Creates a new swerve drivebase subsystem.  This will handle kinematics and
   * odometry. This also handles individual module control; it will use ChassisSpeeds objects
   * given by commands to constantly update wheel position and speed.*/
  public swerveBase() {
    
    // Swerve base kinematics object
    kinematics = new SwerveDriveKinematics(
      new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
      new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
      new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
      new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
    );
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lasttime = 0;
      // Swerve odometry
      odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));
    }

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

    // Get the SparkMAX integrated PIDF controllers
    driveControllerFL = driveFL.getPIDController();
    driveControllerFR = driveFR.getPIDController();
    driveControllerBL = driveBL.getPIDController();
    driveControllerBR = driveBR.getPIDController();

    steerControllerFL = steerFL.getPIDController();
    steerControllerFR = steerFR.getPIDController();
    steerControllerBL = steerBL.getPIDController();
    steerControllerBR = steerBR.getPIDController();

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

    driveControllerFL.setP(Drivebase.VELOCITY_KP);
    driveControllerFR.setP(Drivebase.VELOCITY_KP);
    driveControllerBL.setP(Drivebase.VELOCITY_KP);
    driveControllerBR.setP(Drivebase.VELOCITY_KP);

    driveControllerFL.setI(Drivebase.VELOCITY_KI);
    driveControllerFR.setI(Drivebase.VELOCITY_KI);
    driveControllerBL.setI(Drivebase.VELOCITY_KI);
    driveControllerBR.setI(Drivebase.VELOCITY_KI);

    driveControllerFL.setD(Drivebase.VELOCITY_KD);
    driveControllerFR.setD(Drivebase.VELOCITY_KD);
    driveControllerBL.setD(Drivebase.VELOCITY_KD);
    driveControllerBR.setD(Drivebase.VELOCITY_KD);

    driveControllerFL.setFF(Drivebase.VELOCITY_KF);
    driveControllerFR.setFF(Drivebase.VELOCITY_KF);
    driveControllerBL.setFF(Drivebase.VELOCITY_KF);
    driveControllerBR.setFF(Drivebase.VELOCITY_KF);

    driveControllerFL.setIZone(Drivebase.VELOCITY_IZ);
    driveControllerFR.setIZone(Drivebase.VELOCITY_IZ);
    driveControllerBL.setIZone(Drivebase.VELOCITY_IZ);
    driveControllerBR.setIZone(Drivebase.VELOCITY_IZ);
  }

  public void setVelocity(ChassisSpeeds velocity) {
    robotVelocity = velocity;
  }

  @Override
  public void periodic() {
    // This handles drivebase control operations.  Order of operations is as follows:
    // 1. Convert the commanded ChassisSpeeds to SwerveModuleStates via kinematics
    // 2. Optimize the states
    // 3. Pass the optimized states to the azimuth and velocity controllers.

    SmartDashboard.putString("Commanded Robot Velocity", robotVelocity.toString()); 
    moduleStates = kinematics.toSwerveModuleStates(robotVelocity);

    frontLeft = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(steerFLEncoder.getPosition()));
    frontRight = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(steerFREncoder.getPosition()));
    backLeft = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(steerBLEncoder.getPosition()));
    backRight = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(steerBREncoder.getPosition()));

    SmartDashboard.putString("FrontLeft", frontLeft.toString());
    SmartDashboard.putString("FrontRight", frontRight.toString());
    SmartDashboard.putString("BackLeft", backLeft.toString());
    SmartDashboard.putString("BackRight", backRight.toString());

    steerControllerFL.setReference(frontLeft.angle.getDegrees(), ControlType.kPosition);
    steerControllerFR.setReference(frontRight.angle.getDegrees(), ControlType.kPosition);
    steerControllerBL.setReference(backLeft.angle.getDegrees(), ControlType.kPosition);
    steerControllerBR.setReference(backLeft.angle.getDegrees(), ControlType.kPosition);

    driveControllerFL.setReference(frontLeft.speedMetersPerSecond, ControlType.kVelocity);
    driveControllerFR.setReference(frontRight.speedMetersPerSecond, ControlType.kVelocity);
    driveControllerBL.setReference(backLeft.speedMetersPerSecond, ControlType.kVelocity);
    driveControllerBR.setReference(backRight.speedMetersPerSecond, ControlType.kVelocity);
  }

  @Override
  public void simulationPeriodic() {
    angle += kinematics.toChassisSpeeds(frontLeft, frontRight, backLeft, backRight).omegaRadiansPerSecond
      * (timer.get() - lasttime);
    lasttime = timer.get();

    odometry.update(
      new Rotation2d(angle),
      frontLeft,
      frontRight,
      backLeft,
      backRight);
    
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field);
  }
}
