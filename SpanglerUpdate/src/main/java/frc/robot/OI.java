package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private XboxController driverController = new XboxController(0);
    private XboxController weaponsController = new XboxController(1);

    public boolean auto_collector_deploy = false;
    public boolean auto_shooting = false;
    public double auto_shooting_speed = 0;
    public double auto_collect_speed = 0;

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

  public boolean getBackwardsButtonPressed() {
		return weaponsController.getRightBumper();
	}

  /**
     * Get the roller rotation speed commanded by the driver
     * @return -1 for full speed backward, +1 for full speed forward
     */
	public double getGroundPickUpRollerAxis() {
		if (auto_collect_speed == 0) {
			return weaponsController.getLeftTriggerAxis();
		} else {
			return auto_collect_speed;
		}
	}
	public double getHumanPlayerStationPickUpRollerAxis() {
		return weaponsController.getRightTriggerAxis();
	}

	/**
	 * Ask if the driver wants ground pick-up to be deployed
	 * @return
	 */
	public boolean getGroundPickUpDeployed() {
		if (auto_collector_deploy) {
			auto_collector_deploy = false;
			return true;
		} else {
			return weaponsController.getXButton();
		}
	}

  public boolean getShooterButton() {
		if (auto_shooting) {
			return true;
		}
		else {
			return weaponsController.getYButton();
		}
	}
}
