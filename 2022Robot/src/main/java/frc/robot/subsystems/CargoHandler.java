// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CargoHandler extends SubsystemBase {
  private CANSparkMax collector, collector_2, singulator, singulator_2, indexer, shooter, shooter_2,
    shooterRoller, shooterRoller_2;
  private ColorSensorV3 colorSensor;
  private ColorMatch colorMatcher;
  private Color RedTarget, BlueTarget;

  private I2C.Port i2cport = Port.kOnboard;

  private String COLOR_RED = "RED";
  private String COLOR_BLUE = "BLUE";
  private String COLOR_OTHER = "OTHER";

  public CargoHandler() {
    collector = new CANSparkMax(Constants.COLLECTOR_LEAD, MotorType.kBrushless);
    collector_2 = new CANSparkMax(Constants.COLLECTOR_FOLLOW, MotorType.kBrushless);
    singulator = new CANSparkMax(Constants.SINGULATOR_LEAD, MotorType.kBrushless);
    singulator_2 = new CANSparkMax(Constants.SINGULATOR_FOLLOW, MotorType.kBrushless);
    indexer = new CANSparkMax(Constants.INDEXER_MOTOR, MotorType.kBrushless);
    shooter = new CANSparkMax(Constants.SHOOTER_LEAD, MotorType.kBrushless);
    shooter_2 = new CANSparkMax(Constants.SHOOTER_FOLLOW, MotorType.kBrushless);
    shooterRoller = new CANSparkMax(Constants.SHOOTER_ROLLER_LEAD, MotorType.kBrushless);
    shooterRoller_2 = new CANSparkMax(Constants.SHOOTER_ROLLER_FOLLOW, MotorType.kBrushless);

    collector_2.follow(collector, true);
    singulator_2.follow(singulator, true);
    shooter_2.follow(shooter, true);
    shooterRoller_2.follow(shooterRoller, true);

    colorSensor = new ColorSensorV3(i2cport);
    colorMatcher = new ColorMatch();

    RedTarget = Color.kRed;
    BlueTarget = Color.kBlue;

    colorMatcher.addColorMatch(RedTarget);
    colorMatcher.addColorMatch(BlueTarget);
  }

  public String getCargoColor() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    if (match.color == RedTarget) {
      return COLOR_RED;
    } else if (match.color == BlueTarget) {
      return COLOR_BLUE;
    } else {
      return COLOR_OTHER;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Cargo Color", getCargoColor());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
