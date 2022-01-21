// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveBase extends SubsystemBase {

  private CANSparkMax leftPod, rightPod, l2, r2;

  public DriveBase() {
    leftPod = new CANSparkMax(Constants.LEFT_LEAD, MotorType.kBrushless);
    rightPod = new CANSparkMax(Constants.RIGHT_LEAD, MotorType.kBrushless);
    l2 = new CANSparkMax(Constants.LEFT_F, MotorType.kBrushless);
    r2 = new CANSparkMax(Constants.RIGHT_F, MotorType.kBrushless);

    l2.follow(leftPod);
    r2.follow(rightPod);

    rightPod.setInverted(true);
  }

  public void driveWithJoysticks() {
    double x = RobotContainer.oi.getForwardAxis();
    double y = RobotContainer.oi.getTurnAxis();
    x = Math.pow(x, 3);
    y = Math.pow(y, 3);
    leftPod.set(x - y);
    rightPod.set(x + y);
  }

  /**
   * Toggle motor brakes
   */
  public void setBreaks(boolean enabled) {
    leftPod.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    rightPod.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
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
