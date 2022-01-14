/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase {
  PowerDistribution pdp;
  double restingVoltage = 12;

  public Telemetry() {
    pdp = new PowerDistribution(0, ModuleType.kCTRE);
  }

  public double getVoltage() {
    return pdp.getVoltage();
  }

  public double getRestingVoltage() {
    double current = getCurrent();
    if (current == 0) {
      restingVoltage = getVoltage();
    }
    return restingVoltage;
  }

  public double getVoltageDrop() {
    double voltageDrop = getRestingVoltage() - getVoltage();
    if (voltageDrop < 0) {
      return 0;
    } else {
      return voltageDrop;
    }
  }

  public double getCurrent() {
    return pdp.getTotalCurrent();
  }

  public double getResistance() {
    double voltage = getVoltageDrop();
    double current = getCurrent();
    if (current > 0) {
      return voltage / current;
    } else {
      return 0;
    }
  }

  public double getTemp() {
    return pdp.getTemperature();
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
