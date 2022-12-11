// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;

public class swerveBase extends SubsystemBase {

  private SwerveModule[] swerveModules;
  private PigeonIMU imu;
  
  private SwerveDriveOdometry odometry;
  private Field2d field = new Field2d();

  private double angle, lasttime;

  private Timer timer;

  /** Creates a new swerve drivebase subsystem.  This will handle kinematics and
   * odometry. This also handles individual module control; it will use ChassisSpeeds objects
   * given by commands to constantly update wheel position and speed.*/
  public swerveBase() {
    
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lasttime = 0;
    } else {
      imu = new PigeonIMU(Drivebase.PIGEON);
      imu.configFactoryDefault();
      zeroGyro();
    }

    odometry = new SwerveDriveOdometry(Drivebase.KINEMATICS, getYaw());

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Drivebase.Mod0.CONSTANTS),
      new SwerveModule(1, Drivebase.Mod1.CONSTANTS),
      new SwerveModule(2, Drivebase.Mod2.CONSTANTS),
      new SwerveModule(3, Drivebase.Mod3.CONSTANTS),
    };
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), 
      translation.getY(), 
      rotation, 
      getYaw()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    );
    SmartDashboard.putString("RobotVelocity", velocity.toString());
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    for (SwerveModule module : swerveModules) {
      SmartDashboard.putString("Module" + module.toString(), swerveModuleStates[module.moduleNumber].toString());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getYaw());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  public void zeroGyro() {
    if (Robot.isReal()) {
      imu.setYaw(0);
    } else {
      angle = 0;
    }
  }

  public Rotation2d getYaw() {
    if (Robot.isReal()) {
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    } else {
      return new Rotation2d(angle);
    }
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getStates());

    if (!Robot.isReal()) {
      angle += Drivebase.KINEMATICS.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lasttime);
      lasttime = timer.get();

      field.setRobotPose(odometry.getPoseMeters());
      SmartDashboard.putData("Field", field);
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
