// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber_Properties;

public class Climber extends SubsystemBase {

  private Solenoid pistons;
  private CANSparkMax leftLead;
  private CANSparkMax rightLead;

  public Climber() {
    pistons = new Solenoid(PneumaticsModuleType.REVPH, Climber_Properties.CLIMBER_PNEUMATICS_ID);
    leftLead = new CANSparkMax(Climber_Properties.LEFT_LEAD, MotorType.kBrushless);
    rightLead = new CANSparkMax(Climber_Properties.RIGHT_LEAD, MotorType.kBrushless);
  }

  /**
   * Toggle motor brakes
   */
  public void setBreaks(boolean enabled) {
    leftLead.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    rightLead.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setSpeed(double speed) {
    leftLead.set(-speed);
    rightLead.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPistons(boolean extend) {
    pistons.set(extend);
  }
}
