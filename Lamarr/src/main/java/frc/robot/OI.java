package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
	public int[] ClimberStageOneSteps = {0, 0, 0, 0, 0};
	public int[] ClimberStageTwoSteps = {0, 0, 0, 0, 0};
	public int[] ClimberStageThreeSteps = {0, 0, 0, 0, 0};
	public int[] ClimberStageFourSteps = {0, 0, 0, 0, 0};

	public XboxController driverController = new XboxController(0);
	public XboxController weaponsController = new XboxController(1);

	public boolean auto_collector_toggle = false;
	public boolean auto_shooting = false;
	public boolean manual_shooting_high = false;
	public double auto_shooting_speed = 0;
	public double auto_roller_speed = 0;
	public double auto_collect_speed = 0;

	/** Describes which of the controlleres you're referring to */
	public enum Controller {
		DRIVER,
		WEAPONS, // Weapons operator
	}

	// System timestamps after which we want each rumbler to be turned off
	private double driverLeftRumbleStopTime = 0;
	private double driverRightRumbleStopTime = 0;
	private double weaponsLeftRumbleStopTime = 0;
	private double weaponsRightRumbleStopTime = 0;

       /*
    Update this whenever buttons are rebound to avoid double-binding a button
    Current Button Mappings:
      Driver:
        Left Stick up/down -> robot fwd/back
        Right Stick left/right -> robot turn
        Left Bumper -> Autocollect
        Right Bumper -> Airbrakes
        Y -> Auto shoot high hub
        A -> Auto shoot low hub
        POV DOWN -> Auto Climb 1
        POV RIGHT -> Auto Climb 2
        POV UP -> Auto Climb 3
        POV LEFT -> Auto Climb 4
      Gunner:
        Left Trigger -> Run Collector
        Right Trigger -> Manually Eject Cargo
        Left Bumper -> Unspool climber winches
        Right Bumper -> Spool climber winches
        Start -> Deploy climber to defensive mode
        Back -> Retract climber from defensive mode
        Y -> Manual shoot high
		B -> Manual shoot low
        A -> Toggle climber pneumatics
        X -> Deploy/undeploy (toggle) collector
	*/

	public boolean getShootertest() {
		return driverController.getStartButton();
	}

	public boolean getWeaponsLeftTriggerPulled() {
		if(weaponsController.getLeftTriggerAxis() > 0.25)
			return true;
		else
			return false;
	}

	public boolean getWeaponsRightTriggerPulled() {
		if(weaponsController.getRightTriggerAxis() > 0.25)
			return true;
		else
			return false;
	}

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

	/**
	 * Get the roller rotation speed commanded by the driver
	 * 
	 * @return -1 for full speed backward, +1 for full speed forward
	 */
	public double getGroundPickUpRollerAxis() {
		if (auto_collect_speed == 0) {
			return weaponsController.getLeftTriggerAxis();
		} else {
			return auto_collect_speed;
		}
	}

	/**
	 * Ask if the driver wants ground pick-up to be deployed
	 * 
	 * @return
	 */
	public boolean getGroundPickUpDeployed() {
		if (auto_collector_toggle) {
			auto_collector_toggle = false;
			return true;
		} else {
			return weaponsController.getXButton();
		}
	}

	public boolean getClimberUnspoolButton() {
		return weaponsController.getLeftBumper();
	}

	public boolean getClimberSpoolButton() {
		return weaponsController.getRightBumper();
	}

	public boolean getBrakesButton() {
		return driverController.getRightBumper();
	}

	public boolean getClimberButton() {
		return weaponsController.getAButton();
	}

	public boolean getShooterButton() {
		if (auto_shooting) {
			return true;
		} else if (weaponsController.getYButton()){
			manual_shooting_high = true;
			return true;
		} else if (weaponsController.getBButton()) {
			manual_shooting_high = false;
			return true;
		} else {
			return false;
		}
	}

	public boolean getEjectButton() {
		if(weaponsController.getRightTriggerAxis() > 0.15)
			return true;
		else
			return false;
	}

	/**
	 * Rumble a controller.
	 * Note that you may have overlapping low- and high-pitched rumbles
	 * 
	 * @param controller which controller to rumble
	 * @param side       right of left side. Note that the left side has a lower RPM
	 *                   and what feels like a heavier weight compared to the right.
	 * @param severity   how strongly to rumble, between 0.0 and 1.0
	 * @param duration   how long, in seconds, the rumble should last
	 */
	public void Rumble(Controller controller, XboxController.RumbleType side, double severity, double duration) {
		XboxController stick = null;
		switch (controller) {
			case DRIVER:
				stick = driverController;
				switch (side) {
					case kRightRumble:
						driverRightRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
					case kLeftRumble:
						driverLeftRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
				}
				break;
			case WEAPONS:
				stick = weaponsController;
				switch (side) {
					case kRightRumble:
						weaponsRightRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
					case kLeftRumble:
						weaponsLeftRumbleStopTime = Timer.getFPGATimestamp() + duration;
						break;
				}
				break;
		}

		stick.setRumble(side, severity);
	}

	/**
	 * Cease all rumbling
	 */
	public void CancelRumble() {
		CancelRumble(Controller.DRIVER);
		CancelRumble(Controller.WEAPONS);
	}

	/**
	 * Cease all rumbling on a controller
	 */
	public void CancelRumble(Controller controller) {
		XboxController stick = null;
		switch (controller) {
			case DRIVER:
				stick = driverController;
				driverRightRumbleStopTime = Timer.getFPGATimestamp() - 1;
				driverLeftRumbleStopTime = Timer.getFPGATimestamp() - 1;
				break;
			case WEAPONS:
				stick = weaponsController;
				weaponsRightRumbleStopTime = Timer.getFPGATimestamp() - 1;
				weaponsLeftRumbleStopTime = Timer.getFPGATimestamp() - 1;
				break;
		}

		stick.setRumble(XboxController.RumbleType.kRightRumble, 0);
		stick.setRumble(XboxController.RumbleType.kRightRumble, 0);
	}

	public void periodic() {
		if (driverLeftRumbleStopTime <= Timer.getFPGATimestamp()) {
			driverController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
		}
		if (driverRightRumbleStopTime <= Timer.getFPGATimestamp()) {
			driverController.setRumble(XboxController.RumbleType.kRightRumble, 0);
		}
		if (weaponsLeftRumbleStopTime <= Timer.getFPGATimestamp()) {
			weaponsController.setRumble(XboxController.RumbleType.kLeftRumble, 0);
		}
		if (weaponsRightRumbleStopTime <= Timer.getFPGATimestamp()) {
			weaponsController.setRumble(XboxController.RumbleType.kRightRumble, 0);
		}
	}
}