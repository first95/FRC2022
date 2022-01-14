/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Telemetry;

/**
 * An example command. You can replace me with your own command.
 */
public class PDPLogger extends CommandBase {

  double charge = 0;
  long lastTime = System.currentTimeMillis();
  List<Double> resistanceSample = new ArrayList<Double>();
  double sum = 0;

  private Telemetry m_Telemetry;

  public PDPLogger(Telemetry telemetry) {
    m_Telemetry = telemetry;
    addRequirements(telemetry);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    long now = System.currentTimeMillis();
    long elapsedTime = now - lastTime;
    double resistance = m_Telemetry.getResistance();
    lastTime = now;
    double elapsedTimeSec = (double) elapsedTime / 1000;
    charge = charge + ((elapsedTimeSec * m_Telemetry.getCurrent()) / 3600);
    SmartDashboard.putNumber("Current", m_Telemetry.getCurrent());
    SmartDashboard.putNumber("Charge", charge);

    if (resistanceSample.size() < 10) {
      resistanceSample.add(resistance);
      sum += resistance;
    } else {
      sum = sum - resistanceSample.get(0);
      sum += resistance;
      resistanceSample.remove(0);
      resistanceSample.add(resistance);
    }
    double average = sum / 10;
    SmartDashboard.putNumber("Resistance", average);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

}
