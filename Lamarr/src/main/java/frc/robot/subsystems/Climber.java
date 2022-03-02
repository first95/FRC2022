// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber_Properties;

public class Climber extends SubsystemBase {

  private Solenoid pistons;
  private CANSparkMax leftLead;
  private CANSparkMax rightLead;
  private SparkMaxPIDController leftController, rightController;

  public Climber() {
    pistons = new Solenoid(PneumaticsModuleType.REVPH, Climber_Properties.CLIMBER_PNEUMATICS_ID);
    leftLead = new CANSparkMax(Climber_Properties.LEFT_LEAD, MotorType.kBrushless);
    rightLead = new CANSparkMax(Climber_Properties.RIGHT_LEAD, MotorType.kBrushless);

    leftLead.setSmartCurrentLimit(40);
    rightLead.setSmartCurrentLimit(40);
    rightLead.setInverted(true);

    leftController = leftLead.getPIDController();
    rightController = rightLead.getPIDController();

    applyPositionPidConsts();
  }

  /**
   * Toggle motor brakes
   */
  public void setBreaks(boolean enabled) {
    leftLead.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    rightLead.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setSpeed(double speed) {
    leftLead.set(speed);
    rightLead.set(speed);
  }

  public void setEncoderPosition(double position) {
    leftLead.getEncoder().setPosition(0);
    rightLead.getEncoder().setPosition(0);
  }

  public void travelDistance(double rotations) {
    leftController.setReference(rotations * Climber_Properties.NEW_GEAR, com.revrobotics.CANSparkMax.ControlType.kPosition);
    rightController.setReference(rotations * Climber_Properties.NEW_GEAR, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }

  public BooleanSupplier hasLeftReachedReference(double reference) {
    return () -> { return leftLead.getEncoder().getPosition() + 1.5 > (reference * Climber_Properties.NEW_GEAR)
    && leftLead.getEncoder().getPosition() -1.5 < (reference * Climber_Properties.NEW_GEAR); };
  }

  public BooleanSupplier hasRightReachedReference(double reference) {
    return () -> { return rightLead.getEncoder().getPosition() + 1.5 > (reference * Climber_Properties.NEW_GEAR) 
    && rightLead.getEncoder().getPosition() -1.5 < (reference * Climber_Properties.NEW_GEAR); };
  }

  public void applyPositionPidConsts() {

    // Display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", 0.0);
    // SmartDashboard.putNumber("I Gain", 0.0);
    // SmartDashboard.putNumber("D Gain", 0.0);
    // SmartDashboard.putNumber("I Zone", 0.0);
    // SmartDashboard.putNumber("Feed Forward", 0.0);
    // SmartDashboard.putNumber("Max Output", 0.0);
    // SmartDashboard.putNumber("Min Output", 0.0);

    // Read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // If PID coefficients on SmartDashboard have changed, write new values to
    // controller
    leftController.setP(Constants.Climber_Properties.kP);
    rightController.setP(Constants.Climber_Properties.kP);

    leftController.setI(Constants.Climber_Properties.kI);
    rightController.setI(Constants.Climber_Properties.kI);

    // leftController.setD(d);
    // rightController.setD(d);

    // leftController.setIZone(iz);
    // rightController.setIZone(iz);

    // leftController.setFF(ff);
    // rightController.setFF(ff);

    leftController.setOutputRange(-0.75, 0.75);
    rightController.setOutputRange(-0.75, 0.75);

    // leftController.setOutputRange(-Constants.Climber_Properties.MAX_CLIMBER_SPEED, Constants.Climber_Properties.MAX_CLIMBER_SPEED);
    // rightController.setOutputRange(-Constants.Climber_Properties.MAX_CLIMBER_SPEED, Constants.Climber_Properties.MAX_CLIMBER_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Left Enc", leftLead.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Right Enc", rightLead.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void togglePistons() {
    pistons.set(!pistons.get());
  }
}
