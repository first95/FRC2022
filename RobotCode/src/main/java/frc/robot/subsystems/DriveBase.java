
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The DriveBase subsystem incorporates the sensors and actuators attached to
 * the robot's chassis. These include two 3-motor drive pods.
 */
public class DriveBase extends SubsystemBase {

	private CANSparkMax leftPod, leftfollow, rightPod, rightfollow;

	private Solenoid shifter;

	private TalonSRX sucker;



	// Mode for the gearshift, as set by the auto moves
	public enum GearShiftMode {
		LOCK_HIGH_GEAR, LOCK_LOW_GEAR, AUTOSHIFT,
	}


	public DriveBase() {
		super();

		// Note that one pod must be inverted, since the gearbox assemblies are
		// rotationally symmetrical
		leftPod = new CANSparkMax(Constants.LEFT_LEAD, MotorType.kBrushless);
		leftfollow = new CANSparkMax(Constants.LEFT_F, MotorType.kBrushless);
		rightPod = new CANSparkMax(Constants.RIGHT_LEAD, MotorType.kBrushless);
		rightfollow = new CANSparkMax(Constants.RIGHT_F, MotorType.kBrushless);
		leftPod.restoreFactoryDefaults();
		leftfollow.restoreFactoryDefaults();
		rightPod.restoreFactoryDefaults();
		rightfollow.restoreFactoryDefaults();
		leftfollow.follow(leftPod);
		rightfollow.follow(rightPod);

		rightPod.setInverted(true);

		// shifter = new Solenoid(Constants.SHIFTER_SOLENOID_NUM); // 2020 API Version
		shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHIFTER_SOLENOID_NUM);

		sucker = new TalonSRX(Constants.SUCKER);
		setGear(false);
	}


	/**
	 * Turn dynamic braking on or off
	 * 
	 * @param isEnabled true to brake, false to freewheel
	 */
	public void brake(boolean isEnabled) {
		leftPod.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
		rightPod.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
		leftfollow.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
		rightfollow.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);

	}

	/**
	 * Apply the appropriate gear decided by the auto/manual shifting logic
	 * 
	 * @param isHighGear true for high gear, false for low gear
	 */
	private void setGear(boolean isHighGear) {
		// System.out.println("Shifting to " + (isHighGear? "high":"low") + " gear");
		if (shifter != null) {
			shifter.set(isHighGear);
		}
	}

	/**
	 * 
	 * @return true for high gear, false for low gear
	 */
	public boolean getGear() {
		// True in high gear
		// False in low gear
		if (shifter != null) {
			return shifter.get();
		} else {
			return false;
		}
	}

	/**
	 * Set the power on the sucker
	 * 
	 * @param power 0 for off, 1 for full on
	 */
	public void SetSuckerPower(double power) {
		// true for high gear, false for low gear
		if (getGear()) {
			// if in high gear, set power to 0
			System.out.println("Nuh uh - can't use sucker in high gear!");
			power = 0;
		}
		sucker.set(ControlMode.PercentOutput, power);
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
