
package frc.robot.components;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Each DrivePod represents one of the sides of the robot. Each pod consists of
 * 3 drive motors slaved into one gearbox, along with its shifter and shaft
 * encoder.
 */

public class DrivePod {
	// Measured 2/13/18 on practice robot on "field" carpet
	private static final double ENCODER_TICKS_PER_INCH = 23840.0 / (4 * 12); // 25560.0 / (4 * 12);

	private IMotorControllerEnhanced leader, follower1, follower2;
	private String name;
	private double twiddle = 1.0; // This value is used to force SmartDashboard line graphs to update by slightly changing the value

	// Provide the CAN addresses of the three motor controllers.
	// Set reverse to true if positive throttle values correspond to moving the
	// robot backwards.
	// (This is to account for the way the drive pods are mounted in a rotationally
	// symmetric way.)
	// Name is for feedback on the SmartDashboard - likely "left" or "right"
	public DrivePod(String name, int leaderCanNum, int follower1CanNum, int follower2CanNum, boolean reverse) {
		this.name = name;

		this.leader = new TalonSRX(leaderCanNum);
		this.follower1 = new TalonSRX(follower1CanNum);
		this.follower2 = new TalonSRX(follower2CanNum);
		
		// Tell the followers to follow the leader
		follower1.set(ControlMode.Follower, leaderCanNum);
		follower2.set(ControlMode.Follower, leaderCanNum);

		leader.setInverted(reverse);
		follower1.setInverted(reverse);
		follower2.setInverted(reverse);

		init();
	}

	private void init() {
		// Leaders have quadrature encoders connected to their inputs
		leader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PID_IDX, Constants.CAN_TIMEOUT_MS);
		leader.setSensorPhase(true);

		leader.configForwardSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);
		leader.configReverseSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);

	}

	public double getPositionInches() {
		return leader.getSelectedSensorPosition(Constants.PID_IDX) / ENCODER_TICKS_PER_INCH;
	}


	public double getTargetVelocityInchesPerSecond() {
		if (getControlMode() == ControlMode.Velocity) {
			double speedTicksPer100ms = leader.getClosedLoopTarget(Constants.PID_IDX);
			return (speedTicksPer100ms / ENCODER_TICKS_PER_INCH) * 10.0;
		} else {
			return 0;
		}
	}

	private ControlMode getControlMode() {
		return leader.getControlMode();
	}

	public void log() {
		if (twiddle > 1.0) {
			twiddle = 1.0;
		} else {
			twiddle = 1.000001;
		}
		// Anything we wanna see on the SmartDashboard, put here. Use "name",
		// which should be "left" or "right".
		SmartDashboard.putNumber(name + " position (in)", twiddle * getPositionInches());
		SmartDashboard.putNumber(name + " velocity (inps)", twiddle * getEncoderVelocityFeetPerSecond() * 12.0);
		SmartDashboard.putNumber(name + " target velocity (inps)", twiddle * getTargetVelocityInchesPerSecond());
		SmartDashboard.putNumber("BUSvoltage", leader.getBusVoltage());
		SmartDashboard.putNumber("OutputVoltage", leader.getMotorOutputVoltage());
		SmartDashboard.putNumber("eIZone",
				leader.configGetParameter(314, Constants.CAN_ORDINAL_SLOT0, Constants.CAN_TIMEOUT_MS));
		SmartDashboard.putNumber("eIValue",
				leader.configGetParameter(311, Constants.CAN_ORDINAL_SLOT0, Constants.CAN_TIMEOUT_MS));
	}

	// Throttle here is the traditional value, between -1.0 and 1.0, indicating
	// how much power should
	// be applied to the motor. It corresponds well to speed.
	public void setThrottle(double throttle) {
		// This is the only set...() method where we don't need to call either
		// applySpeedPidConsts() or applyPositionPidConsts().
		leader.set(ControlMode.PercentOutput, throttle);
		// followers follow
	}

	// Max speed back and forward, always make this number positve when setting it.
	public void setMaxSpeed(double maxSpeed) {
		leader.configPeakOutputForward(maxSpeed, Constants.CAN_TIMEOUT_MS);
		leader.configPeakOutputReverse(-maxSpeed, Constants.CAN_TIMEOUT_MS);
	}

	public void setVoltageRamp(double rampRate) {
		leader.configOpenloopRamp(rampRate, Constants.CAN_TIMEOUT_MS);
	}

	public void enableBrakeMode(boolean isEnabled) {
		leader.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		follower1.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		follower2.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
	}

	public double getQuadEncPos() {
		return leader.getSelectedSensorPosition(Constants.PID_IDX);
	}

	public double getEncoderVelocityFeetPerSecond() {
		return (leader.getSelectedSensorVelocity(Constants.PID_IDX)) * (1 / (ENCODER_TICKS_PER_INCH * 12)) * (10 / 1);
	}


}
