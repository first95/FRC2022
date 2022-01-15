/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class PowerCellMover extends SubsystemBase {

  DigitalInput SingulatorSensor = new DigitalInput(Constants.SINGULATOR_SENSOR);
  DigitalInput IndexerEntranceSensor = new DigitalInput(Constants.INDEXER_ENTRANCE_SENSOR);
  DigitalInput IndexerLoadedSensor = new DigitalInput(Constants.INDEXER_POWERCELL_LOADED_SENSOR);
  DigitalInput ShooterLoadedSensor = new DigitalInput(Constants.SHOOTER_LOADED_SENSOR);

  private CANSparkMax beltMotor, leader, follower;
  private CANEncoder leaderEncoder;
  private IMotorControllerEnhanced Singulator, SingulatorIntake;
  private TalonSRX rollers;
  public Solenoid deploy;

  public PowerCellMover() {
    super();

    // SparkMax initialization
    beltMotor = new CANSparkMax(Constants.INDEXER_BELT_MOTOR_ID, MotorType.kBrushless);
    leader = new CANSparkMax(Constants.LEADER_SHOOT, MotorType.kBrushless);
    follower = new CANSparkMax(Constants.FOLLOWER_SHOOT, MotorType.kBrushless);

    // Talon initialization
    Singulator = new TalonSRX(Constants.INNER_SINGULATOR_TALON_ID);
    SingulatorIntake = new TalonSRX(Constants.SINGULATOR_INTAKE_TALON_ID);
    rollers = new TalonSRX(Constants.GROUND_PICK_UP_TALON_ID);

    // Solenoid initialization
    deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.GROUND_PICK_UP_SOLENOID_ID);

    init();
  }

  public boolean getSingulatorSensor() {
    // get the current state of the sensor watching for powercells in the singulator
    return (!SingulatorSensor.get());
  }

  public boolean getIndexerEntranceSensor() {
    // get the current state of the sensor watching for powercells where the indexer
    // can first grab them
    return IndexerEntranceSensor.get();
  }

  public boolean getIndexerLoadedSensor() {
    // get the current state of the sensor watching for powercells in the first
    // position fully within the indexer
    return IndexerLoadedSensor.get();
  }

  public boolean getShooterLoadedSensor() {
    // get the current state of the sensor watching for powercells in position to be
    // fired
    return ShooterLoadedSensor.get();
  }

  private void init() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();
    follower.follow(leader, true); // true inverts the follower from the leader

    leaderEncoder = leader.getEncoder();
  }

  /**
   * Run the shooter
   * 
   * @param speed 0 for stationary, 1 for full forward, -1 for full reverse
   */
  public void runShooterOpen(double speed) {
    // System.out.println("Setting shooter speed to " + speed);
    leader.set(speed);
  }

  /**
   * Get speed from shooter (RPM)
   */
  public double getShooterSpeed() {
    return leaderEncoder.getVelocity();
  }

  /**
   * Run the indexer belt
   * 
   * @param speed 0 for stationary, 1 for full forward
   */
  public void runIndexer(double speed) {
    // System.out.println("Setting indexer speed to " + speed);
    beltMotor.set(speed);
    beltMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Set the speed at which the singulator rollers rotate
   */
  public void setSingulatorSpeed(double Speed) {
    Singulator.set(ControlMode.PercentOutput, Speed);

  }

  /**
   * Set the speed at which the roller rotates at
   */
  public void setRollerSpeed(double speed) {
    if (deploy.get()) {
      rollers.set(ControlMode.PercentOutput, speed);
      // System.out.println("rollers have a command");
    } else {
      rollers.set(ControlMode.PercentOutput, 0);
      System.out.println("Can't run ground pickup when not deployed");
    }
  }

  public void setSingulatorIntakeSpeed(double speed) {
    if (deploy.get()) {
      SingulatorIntake.set(ControlMode.PercentOutput, -speed);
    } else {
      SingulatorIntake.set(ControlMode.PercentOutput, 0);
      System.out.println("Can't run ground pickup when not deployed");
    }
  }

  /**
   * If ground pick-up is deployed, retract it; else, deploy it
   */
  public void toggleGroundPickUpDeploy() {
    deploy.set(!deploy.get());
    // System.out.println("it has been deployed");
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
