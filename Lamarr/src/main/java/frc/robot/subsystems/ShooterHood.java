// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CargoHandling;

public class ShooterHood extends SubsystemBase {
  private Solenoid hood;
  public ShooterHood() {
    hood = new Solenoid(PneumaticsModuleType.REVPH, CargoHandling.HOOD_PNEUMATICS_ID);
  }

  public void setHood(boolean up) {
    hood.set(up);
  }

  public boolean getHood() {
    return hood.get();
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