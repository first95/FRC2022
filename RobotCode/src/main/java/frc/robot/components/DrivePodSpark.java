
package frc.robot.components;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Each DrivePod represents one of the sides of the robot. Each pod consists of
 * 2 drive motors slaved into one gearbox, along with its shifter and shaft
 * encoder.
 * 
 * Updated for 2022, REMOVED SHIFTER, EXTERNAL ENCODERS, AND VARIOUS OTHER
 * THINGS
 * TODO: Update libraries to remove deprected components
 * TODO: Generalize class for future seasons (Shifter, external encoder, etc...)
 */

public class DrivePodSpark extends MotorControllerGroup {
	private CANSparkMax leader, follower;
	private boolean inverse;

	private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
	private static final int kCPR = 1024;

	private CANPIDController leaderPidController, followerPidController;

	// PID coefficients
	public double kP = 0.01;
	public double kI = 0;
	public double kD = 0;
	public double kIz = 0;
	public double kFF = 0;
	public double kMaxOutput = 1;
	public double kMinOutput = -1;

	// Provide the CAN addresses of the three motor controllers.
	// Set reverse to true if positive throttle values correspond to moving the
	// robot backwards.
	// (This is to account for the way the drive pods are mounted in a rotationally
	// symmetric way.)
	// Name is for feedback on the SmartDashboard - likely "left" or "right"
	public DrivePodSpark(String name, CANSparkMax leader, CANSparkMax follower, boolean reverse) {
		super(leader, follower);

		this.leader = leader;
		this.follower = follower;

		// Create the default PID controller associated with the leader
		leaderPidController = this.leader.getPIDController();
		followerPidController = this.follower.getPIDController();

		leader.restoreFactoryDefaults();
		follower.restoreFactoryDefaults();

		// Tell the followers to follow the leader
		follower.follow(leader);

		inverse = reverse;
		init();
	}

	private void init() {

		// Set PID coefficients
		leaderPidController.setP(kP);
		followerPidController.setP(kP);
		leaderPidController.setI(kI);
		followerPidController.setI(kI);
		leaderPidController.setD(kD);
		followerPidController.setD(kD);
		leaderPidController.setIZone(kIz);
		followerPidController.setIZone(kIz);
		leaderPidController.setFF(kFF);
		followerPidController.setFF(kFF);
		leaderPidController.setOutputRange(kMinOutput, kMaxOutput);
		followerPidController.setOutputRange(kMinOutput, kMaxOutput);

		// Leaders have quadrature encoders connected to their inputs
		// The following is needed if alternate, non-SparkMax-built-in encoder is used
		// leader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
		// Constants.PID_IDX, Constants.CAN_TIMEOUT_MS);

		// Not sure if the following is needed for SparkMax
		// leader.setSensorPhase(true);

		// leader.configForwardSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);
		// leader.configReverseSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);
	}

	public double getWheelPositions() {
		var position = leader.getEncoder().getPosition() * Constants.METERS_PER_ROTATION;
		return position;
	}

	public void resetWheelPositions() {
		leader.getEncoder().setPosition(0);
		follower.getEncoder().setPosition(0);
	}

	// Throttle here is the traditional value, between -1.0 and 1.0, indicating
	// how much power should
	// be applied to the motor. It corresponds well to speed.
	public void setThrottle(double throttle) {
		// This is the only set...() method where we don't need to call either
		// applySpeedPidConsts() or applyPositionPidConsts().
		if (inverse) {
			leader.set(-1 * throttle);
		} else {
			leader.set(throttle);
		}

		// followers follow
	}

	public void setVoltageRamp(double rampRate) {
		leader.setOpenLoopRampRate(rampRate);
		follower.setOpenLoopRampRate(rampRate);
	}

	public void enableBrakeMode(boolean isEnabled) {
		leader.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
		follower.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
	}

	public double getEncoderVelocityFeetPerSecondSansGear() {
		// NOTE: getVelocity returns velocity in motor unit (default is RPM)
		return leader.getEncoder().getVelocity() * (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI / 60);
	}

	public void applyPositionPidConsts() {
		// Display PID coefficients on SmartDashboard
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
		SmartDashboard.putNumber("I Zone", kIz);
		SmartDashboard.putNumber("Feed Forward", kFF);
		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);

		// Read PID coefficients from SmartDashboard
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);
		double iz = SmartDashboard.getNumber("I Zone", 0);
		double ff = SmartDashboard.getNumber("Feed Forward", 0);
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);

		// If PID coefficients on SmartDashboard have changed, write new values to
		// controller
		if ((p != kP)) {
			leaderPidController.setP(p);
			followerPidController.setP(p);
			kP = p;
		}
		if ((i != kI)) {
			leaderPidController.setI(i);
			followerPidController.setI(i);
			kI = i;
		}
		if ((d != kD)) {
			leaderPidController.setD(d);
			followerPidController.setD(d);
			kD = d;
		}
		if ((iz != kIz)) {
			leaderPidController.setIZone(iz);
			followerPidController.setIZone(iz);
			kIz = iz;
		}
		if ((ff != kFF)) {
			leaderPidController.setFF(ff);
			followerPidController.setFF(ff);
			kFF = ff;
		}
		if ((max != kMaxOutput) || (min != kMinOutput)) {
			leaderPidController.setOutputRange(min, max);
			followerPidController.setOutputRange(min, max);
			kMinOutput = min;
			kMaxOutput = max;
		}
	}

	public void travleDistance(double rotations) {
		// Set the set point
		leaderPidController.setReference(rotations, ControlType.kPosition);
		followerPidController.setReference(rotations, ControlType.kPosition);

		// Display set point and position of motor (in rotations) on SmartDashboard
		SmartDashboard.putNumber("SetPoint", rotations);
	}
}
