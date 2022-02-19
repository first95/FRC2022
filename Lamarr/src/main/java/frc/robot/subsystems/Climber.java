// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    leftController = leftLead.getPIDController();
    rightController = rightLead.getPIDController();
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
    rightLead.set(-speed);
  }

  public void travelDistance(double rotations) {
    leftController.setReference(rotations, com.revrobotics.CANSparkMax.ControlType.kPosition);
    rightController.setReference(rotations, com.revrobotics.CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("SetPoint", rotations);
  }

  public void applyPositionPidConsts() {

    // Display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);

    // Read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // If PID coefficients on SmartDashboard have changed, write new values to
    // controller
    leftController.setP(p);
    rightController.setP(p);

    leftController.setI(i);
    rightController.setI(i);

    leftController.setD(d);
    rightController.setD(d);

    leftController.setIZone(iz);
    rightController.setIZone(iz);

    leftController.setFF(ff);
    rightController.setFF(ff);

    leftController.setOutputRange(min, max);
    rightController.setOutputRange(min, max);
  }

  @Override
  public void periodic() {
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
