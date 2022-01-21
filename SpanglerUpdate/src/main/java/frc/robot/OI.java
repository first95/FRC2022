package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private XboxController driverController = new XboxController(0);
    private XboxController weaponsController = new XboxController(1);

      /**
   * Gets the commanded forward driving speed.
   */
  public double getForwardAxis() {
    return driverController.getLeftY();
  }
  /**
   * Gets the commanded turn rate.
   */
  public double getTurnAxis() {
    return driverController.getRightX();
  }
}
