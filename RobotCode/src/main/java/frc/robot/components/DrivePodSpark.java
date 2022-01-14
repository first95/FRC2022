
package frc.robot.components;

// import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANEncoder;
import com.revrobotics.AlternateEncoderType;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import frc.robot.Constants;
// import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Each DrivePod represents one of the sides of the robot. Each pod consists of
 * 2 drive motors slaved into one gearbox, along with its shifter and shaft
 * encoder.
 */

public class DrivePodSpark {
	// Measured 2/13/18 on practice robot on "field" carpet
	// private static final double ENCODER_TICKS_PER_INCH = 23840.0 / (4 * 12); // 25560.0 / (4 * 12);

	private CANSparkMax leader, follower;
	private String name;
	private double twiddle = 1.0; // This value is used to force SmartDashboard line graphs to update by slightly changing the value
	private boolean inverse;

	private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
	private static final int kCPR = 1024;
	// private CANEncoder leaderEncoder, followerEncoder;

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
	public DrivePodSpark(String name, int leaderCanNum, int followerCanNum, boolean reverse) {
		this.name = name;

		this.leader = new CANSparkMax(leaderCanNum, MotorType.kBrushless);
		this.follower = new CANSparkMax(followerCanNum, MotorType.kBrushless);
		
		// Commenting out the alternate encoder until we have the proper hardware interface
		// to go between the grayhill and the Spark Max
		//m_alternateEncoder = this.leader.getAlternateEncoder(kAltEncType, kCPR);

	    // Create the default encoder associated with the leader
		// leaderEncoder = this.leader.getEncoder();
		// followerEncoder = this.follower.getEncoder();

		// Create the default PID controller associated with the leader
		leaderPidController = this.leader.getPIDController();
		followerPidController = this.follower.getPIDController();

		leader.restoreFactoryDefaults();
		follower.restoreFactoryDefaults();

		// Tell the followers to follow the leader
		follower.follow(leader);

		inverse = reverse;
		// leader.setInverted(reverse);
		// follower.setInverted(reverse);

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
		// leader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PID_IDX, Constants.CAN_TIMEOUT_MS);
		
		// Not sure if the following is needed for SparkMax
		// leader.setSensorPhase(true);

		// leader.configForwardSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);
		// leader.configReverseSoftLimitEnable(false, Constants.CAN_TIMEOUT_MS);

	}

	/*
	public double getPositionInches() {
		return leader.getSelectedSensorPosition(Constants.PID_IDX) / ENCODER_TICKS_PER_INCH;
	}
	*/

	/*
	public double getTargetVelocityInchesPerSecond() {
		if (getControlMode() == ControlMode.Velocity) {
			double speedTicksPer100ms = leader.getClosedLoopTarget(Constants.PID_IDX);
			return (speedTicksPer100ms / ENCODER_TICKS_PER_INCH) * 10.0;
		} else {
			return 0;
		}
	}
*/

	// Possibly no way to eplicitly set mode for SparkMax controller
	// private ControlMode getControlMode() {
	// 	return leader.getControlMode();
	// }

	public void log() {
		if (twiddle > 1.0) {
			twiddle = 1.0;
		} else {
			twiddle = 1.000001;
		}
		// Anything we wanna see on the SmartDashboard, put here. Use "name",
		// which should be "left" or "right".
		/*
		SmartDashboard.putNumber(name + " position (in)", twiddle * getPositionInches());
		SmartDashboard.putNumber(name + " velocity (inps)", twiddle * getEncoderVelocityFeetPerSecond() * 12.0);
		SmartDashboard.putNumber(name + " target velocity (inps)", twiddle * getTargetVelocityInchesPerSecond());
		SmartDashboard.putNumber("BUSvoltage", leader.getBusVoltage());
		SmartDashboard.putNumber("OutputVoltage", leader.getMotorOutputVoltage());
		SmartDashboard.putNumber("eIZone",
				leader.configGetParameter(314, Constants.CAN_ORDINAL_SLOT0, Constants.CAN_TIMEOUT_MS));
		SmartDashboard.putNumber("eIValue",
				leader.configGetParameter(311, Constants.CAN_ORDINAL_SLOT0, Constants.CAN_TIMEOUT_MS));
	*/
			}

	// Throttle here is the traditional value, between -1.0 and 1.0, indicating
	// how much power should
	// be applied to the motor. It corresponds well to speed.
	public void setThrottle(double throttle) {
		// This is the only set...() method where we don't need to call either
		// applySpeedPidConsts() or applyPositionPidConsts().
		if (inverse)
		{
			leader.set(-1 * throttle);
			// follower.set(-1 * throttle);
		}
		else
		{
			leader.set(throttle);
			// follower.set(throttle);
		}
		// followers follow
	}

	// Max speed back and forward, always make this number positve when setting it.
	/*
	public void setMaxSpeed(double maxSpeed) {
		leader.configPeakOutputForward(maxSpeed, Constants.CAN_TIMEOUT_MS);
		leader.configPeakOutputReverse(-maxSpeed, Constants.CAN_TIMEOUT_MS);
	}
	*/

	public void setVoltageRamp(double rampRate) {
		leader.setOpenLoopRampRate(rampRate);
		follower.setOpenLoopRampRate(rampRate);
	}

	public void enableBrakeMode(boolean isEnabled) {
		leader.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
		follower.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
	}

	// public double getQuadEncPos() {
	// 	return leader.getSelectedSensorPosition(Constants.PID_IDX);
	// }

	public double getEncoderVelocityFeetPerSecondSansGear() {
		//return leader.getEncoder().getVelocity()* (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI / 60);
		// getVelocity returns velocity in motor unit (default is RPM)
		return 2;//leaderEncoder.getVelocity()*(Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI / 60)/12;
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
	
		// If PID coefficients on SmartDashboard have changed, write new values to controller
		if((p != kP)) { 
			leaderPidController.setP(p); 
			followerPidController.setP(p);
			kP = p; 
		}
		if((i != kI)) { 
			leaderPidController.setI(i); 
			followerPidController.setI(i);
			kI = i; 
		}
		if((d != kD)) { 
			leaderPidController.setD(d); 
			followerPidController.setD(d);
			kD = d; 
		}
		if((iz != kIz)) { 
			leaderPidController.setIZone(iz);
			followerPidController.setIZone(iz);
			kIz = iz; 
		}
		if((ff != kFF)) { 
			leaderPidController.setFF(ff); 
			followerPidController.setFF(ff);
			kFF = ff; 
		}
		if((max != kMaxOutput) || (min != kMinOutput)) { 
			leaderPidController.setOutputRange(min, max); 
			followerPidController.setOutputRange(min, max);
			kMinOutput = min; kMaxOutput = max;
		}
}

	public void travleDistance(double rotations) {
		
		// Set the set point
		leaderPidController.setReference(rotations, ControlType.kPosition);
		followerPidController.setReference(rotations, ControlType.kPosition);
    
		// Display set point and position of motor (in rotations) on SmartDashboard
		SmartDashboard.putNumber("SetPoint", rotations);
    	// SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
	}
}
