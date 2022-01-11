package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.commands.vision.SetCameraMode;
import frc.robot.commands.vision.ToggleCameraMode;
import frc.robot.oi.JoystickAxisButton;
import frc.robot.oi.XBox360Controller;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionCoprocessor;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * One can use SmartDashboard with a controller plugged in to see which values
 * corrospond
 * to which controls.
 */
public class OI {

	// The robot's subsystems and commands are defined here...
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final VisionCoprocessor m_visionCoprocessor = new VisionCoprocessor();
	private final DriveBase m_driveBase = new DriveBase();

	public final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
	public final SetCameraMode m_setCameraMode = new SetCameraMode(m_visionCoprocessor, false);
	public final ManuallyControlDrivebase m_manuallyControlDrivebase = new ManuallyControlDrivebase(m_driveBase);
	public final ToggleCameraMode m_ToggleCameraMode = new ToggleCameraMode(m_visionCoprocessor);

	// Controllers
	private Joystick driverController = new Joystick(0);
	private Joystick weaponsController = new Joystick(1);

	// Buttons on drive controller
	// public static final int CLIMB_SKIDS_BUTTON = 0;//
	// XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int SWITCH_CAM_VIEW_BUTTON = XBox360Controller.Button.START.Number();
	// Features not presently in use - getRawButton(0) always returns false
	public static final int BUTTON_FORCE_LOW_GEAR = XBox360Controller.Button.LEFT_BUMPER.Number();
	public static final int BUTTON_FORCE_HIGH_GEAR = XBox360Controller.Button.RIGHT_BUMPER.Number();

	// Axes on drive controller
	public static final int DRIVE_FORWARD_AXIS = XBox360Controller.Axis.LEFT_STICK_Y.Number();
	public static final int DRIVE_TURN_AXIS = XBox360Controller.Axis.RIGHT_STICK_X.Number();

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

	public OI() {

		// // Create some buttons
		JoystickButton cameraViewSwitcher = new JoystickButton(driverController, SWITCH_CAM_VIEW_BUTTON);
		cameraViewSwitcher.whenPressed(m_ToggleCameraMode);
		// cameraViewSwitcher.close(); // Don't need this one anymore?

		JoystickAxisButton driverRumblerLeft = new JoystickAxisButton(driverController,
				XBox360Controller.Axis.LEFT_TRIGGER.Number());
		driverRumblerLeft.whenPressed(new RumbleCommand(Controller.DRIVER, Joystick.RumbleType.kLeftRumble, 1, 1.0));

		JoystickAxisButton driverRumblerRight = new JoystickAxisButton(driverController,
				XBox360Controller.Axis.RIGHT_TRIGGER.Number());
		driverRumblerRight.whenPressed(new RumbleCommand(Controller.DRIVER, Joystick.RumbleType.kRightRumble, 1, 1.0));

		JoystickAxisButton weaponsRumblerLeft = new JoystickAxisButton(weaponsController,
				XBox360Controller.Axis.LEFT_TRIGGER.Number());
		weaponsRumblerLeft.whenPressed(new RumbleCommand(Controller.WEAPONS, Joystick.RumbleType.kLeftRumble, 1, 1.0));

		JoystickAxisButton weaponsRumblerRight = new JoystickAxisButton(weaponsController,
				XBox360Controller.Axis.RIGHT_TRIGGER.Number());
		weaponsRumblerRight
				.whenPressed(new RumbleCommand(Controller.WEAPONS, Joystick.RumbleType.kRightRumble, 1, 1.0));

	}

	// There are a few things the OI wants to revisit every time around
	public void visit() {

		// Cancel joystick rumble if necessary
		if (Timer.getFPGATimestamp() > driverLeftRumbleStopTime) {
			driverController.setRumble(RumbleType.kLeftRumble, 0);
		}
		if (Timer.getFPGATimestamp() > driverRightRumbleStopTime) {
			driverController.setRumble(RumbleType.kRightRumble, 0);
		}
		if (Timer.getFPGATimestamp() > weaponsLeftRumbleStopTime) {
			weaponsController.setRumble(RumbleType.kLeftRumble, 0);
		}
		if (Timer.getFPGATimestamp() > weaponsRightRumbleStopTime) {
			weaponsController.setRumble(RumbleType.kRightRumble, 0);
		}
	}

	/**
	 * Get the forward travel rate commanded by the driver
	 * 
	 * @return -1 for full speed backward, +1 for full speed forward
	 */
	public double getForwardAxis() {
		return driverController.getRawAxis(DRIVE_FORWARD_AXIS);
	}

	/**
	 * Get the turn rate commanded by the driver
	 * 
	 * @return -1 for full turn leftward (CCW when looking down at the robot), +1
	 *         for full turn rightward (CW when looking down at the robot), 0 for no
	 *         turn
	 */
	public double getTurnAxis() {
		return driverController.getRawAxis(DRIVE_TURN_AXIS);
	}

	/**
	 * Ask if the driver wants the robot to be in high gear
	 * 
	 * @return
	 */
	public boolean getHighGear() {
		return false; // return driverController.getRawButton(BUTTON_FORCE_HIGH_GEAR);
	}

	/**
	 * Ask if the driver wants the robot to be in low gear
	 * 
	 * @return
	 */
	public boolean getLowGear() {
		return driverController.getRawButton(BUTTON_FORCE_LOW_GEAR);
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
	public void Rumble(Controller controller, Joystick.RumbleType side, double severity, double duration) {
		Joystick stick = null;
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
		Joystick stick = null;
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

		stick.setRumble(Joystick.RumbleType.kRightRumble, 0);
		stick.setRumble(Joystick.RumbleType.kRightRumble, 0);
	}
}
