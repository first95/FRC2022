// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CargoHandling;
import frc.robot.Constants.CargoHandling.CargoColor;

public class CargoHandler extends SubsystemBase {
  private CANSparkMax collector, collector_2, singulator, singulator_2, indexer, shooter, shooter_2,
      shooterRoller, shooterRoller_2;
  private RelativeEncoder shooterEncoder;
  private Solenoid collectorDeploy;

  private ColorSensorV3 colorSensor;
  private DigitalInput indexerLoadedSensor, shooterLoadedSensor;

  private ColorMatch colorMatcher;
  private Color RedTarget, BlueTarget;

  private I2C.Port i2cport = Port.kOnboard;

  private final int SINGULATOR_EMPTY = 100;

  private Alliance currentAlliance = DriverStation.getAlliance();

  public CargoHandler() {
    collector = new CANSparkMax(CargoHandling.COLLECTOR_LEAD, MotorType.kBrushless);
    collector_2 = new CANSparkMax(CargoHandling.COLLECTOR_FOLLOW, MotorType.kBrushless);
    singulator = new CANSparkMax(CargoHandling.SINGULATOR_LEAD, MotorType.kBrushless);
    singulator_2 = new CANSparkMax(CargoHandling.SINGULATOR_FOLLOW, MotorType.kBrushless);
    indexer = new CANSparkMax(CargoHandling.INDEXER_MOTOR, MotorType.kBrushless);
    shooter = new CANSparkMax(CargoHandling.SHOOTER_LEAD, MotorType.kBrushless);
    shooter_2 = new CANSparkMax(CargoHandling.SHOOTER_FOLLOW, MotorType.kBrushless);
    collectorDeploy = new Solenoid(PneumaticsModuleType.REVPH, CargoHandling.COLLECTOR_PNEUMATICS_ID);
    // shooterRoller = new CANSparkMax(CargoHandling.SHOOTER_ROLLER_LEAD,
    // MotorType.kBrushless);
    // shooterRoller_2 = new CANSparkMax(CargoHandling.SHOOTER_ROLLER_FOLLOW,
    // MotorType.kBrushless);

    collector_2.follow(collector, true);
    singulator_2.follow(singulator, true);
    shooter_2.follow(shooter, true);
    // shooterRoller_2.follow(shooterRoller, true);

    indexer.setInverted(true);
    shooter.setInverted(true);

    collector.setIdleMode(IdleMode.kBrake);
    collector_2.setIdleMode(IdleMode.kBrake);
    singulator.setIdleMode(IdleMode.kBrake);
    singulator_2.setIdleMode(IdleMode.kBrake);
    indexer.setIdleMode(IdleMode.kBrake);
    shooter.setIdleMode(IdleMode.kCoast);
    shooter_2.setIdleMode(IdleMode.kCoast);
    // shooterRoller.setIdleMode(IdleMode.kCoast);

    shooterEncoder = shooter.getEncoder();

    colorSensor = new ColorSensorV3(i2cport);
    indexerLoadedSensor = new
      DigitalInput(CargoHandling.INDEXER_LOADED_SENSOR_ID);
    shooterLoadedSensor = new
      DigitalInput(CargoHandling.SHOOTER_LOADED_SENSOR_ID);

    colorMatcher = new ColorMatch();

    RedTarget = Color.kRed;
    BlueTarget = Color.kBlue;

    colorMatcher.addColorMatch(RedTarget);
    colorMatcher.addColorMatch(BlueTarget);
  }

  /**
   * Deply/retract the collector.
   * @param deploy true to deploy, false to retract
   */
  public void toggleCollectorDeploy() {
    collectorDeploy.set(!collectorDeploy.get());
  }
  
  /**
   * Run the collector and singulator at the given speed
   * 
   * @param speed -1 to 1, 0 for stop
   */
  public void runCollector(double speed) {
    collector.set(speed);
    singulator.set(speed * 0.5);
  }

  /**
   * Run the indexer at the given speed
   * 
   * @param speed -1 to 1, 0 for stop
   */
  public void runIndexer(double speed) {
    indexer.set(speed);
  }

  /**
   * Run the shooter at the given speed with no active control
   * 
   * @param speed -1 to 1, 0 for stop
   */
  public void runShooter(double speed) {
    shooter.set(speed);
    // shooterRoller.set(speed);
  }

  public CargoColor getCargoColor() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    if (colorSensor.getProximity() < SINGULATOR_EMPTY) {
      return CargoColor.NONE;

    } else if (((match.color == RedTarget) && (currentAlliance == Alliance.Red))
        || ((match.color == BlueTarget) && (currentAlliance == Alliance.Blue))) {
      return CargoColor.RIGHT;
    } else {
      return CargoColor.WRONG;
    }
  }

  public boolean getIndexerLoaded() {
    return indexerLoadedSensor.get();
  }

  public boolean getShooterLoaded() {
    return shooterLoadedSensor.get();
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Indexer Loaded", getIndexerLoaded());
    SmartDashboard.putBoolean("Shooter Loaded", getShooterLoaded());
    switch (getCargoColor()) {
      case NONE:
      SmartDashboard.putString("Color", "NONE");
      break;
      case RIGHT:
      SmartDashboard.putString("Color", "RIGHT");
      break;
      case WRONG:
      SmartDashboard.putString("Color", "WRONG");
      break;
    }
    SmartDashboard.putNumber("rawDist", colorSensor.getProximity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}