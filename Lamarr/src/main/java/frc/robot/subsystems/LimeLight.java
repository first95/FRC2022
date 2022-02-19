// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

/* 
Pipelines:
  0: Detect blue balls
  1. Detect red balls
*/

public class LimeLight extends SubsystemBase {
  private final NetworkTable limelight_target_data;
  private double tv, tx, ty, distance, floorDistance, tshort, correctedAngle, angularHeight;
  private String hostname;

  public LimeLight(String hostname) {
    this.hostname = hostname;
    limelight_target_data = NetworkTableInstance.getDefault().getTable("limelight-" + hostname);

    limelight_target_data.getEntry("pipeline").setNumber(0);
  }

  public void SetTeamColor(DriverStation.Alliance teamAlliance) {
    if (teamAlliance == DriverStation.Alliance.Blue)
      limelight_target_data.getEntry("pipeline").setNumber(0);
    else if (teamAlliance == DriverStation.Alliance.Red)
      limelight_target_data.getEntry("pipeline").setNumber(1);
    else
      limelight_target_data.getEntry("pipeline").setNumber(0);
  }

  @Override
  public void periodic() {
    tv = limelight_target_data.getEntry("tv").getDouble(0.0);
    tx = limelight_target_data.getEntry("tx").getDouble(0.0);
    ty = limelight_target_data.getEntry("ty").getDouble(0.0);
    tshort = limelight_target_data.getEntry("tshort").getDouble(0.0);
    angularHeight = tshort * Vision.DEGREES_PER_PIXEL;
    correctedAngle = Vision.CAM_TILT_DEGREES + ty;

    distance = Vision.TARGET_HEIGHT_INCHES / Math.sin(Math.toRadians(correctedAngle));

    floorDistance = Vision.TARGET_HEIGHT_INCHES / Math.tan(Math.toRadians(correctedAngle));

    SmartDashboard.putNumber(hostname + "-Bearing", tx);
    SmartDashboard.putNumber(hostname + "-LimelightY", ty);
    SmartDashboard.putNumber(hostname + "-Target Valid?", tv);
    SmartDashboard.putNumber(hostname + "-Range (in)", distance);
    SmartDashboard.putNumber(hostname + "-Horiz. Range", floorDistance);
  }

  public void setVisionPipeline(int pipeline) {
    if (pipeline < 0 || pipeline > 9) {
      System.out.println("Pipeline number invalid!");
      return;
    }

    limelight_target_data.getEntry("pipeline").setNumber(pipeline);
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTV() {
    return tv;
  }

  public double getDistanceToTarg() {
    return distance;
  }

  public double getFloorDistanceToTarg() {
    return floorDistance;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
