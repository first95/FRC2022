/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private Solenoid deploy;

  public Climber() {
    super();

    // deploy = new Solenoid(Constants.CLIMBER_SOLENOID_NUM); // 2020 API Version
    deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMBER_SOLENOID_NUM);

    // System.out.println("hardware is set");
  }

  /**
   * If climber is deployed, retract it; else, deploy it
   */
  public void toggleClimberDeploy() {
    deploy.set(!deploy.get());
    System.out.println("climber deploy has been toggled");
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
