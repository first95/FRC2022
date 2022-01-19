
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.components.DrivePodSpark;

/**
 * The DriveBase subsystem incorporates the sensors and actuators attached to
 * the robot's chassis. These include two 3-motor drive pods.
 */
public class DriveBase extends SubsystemBase {
	private DrivePodSpark leftPod;
	private DrivePodSpark rightPod;

	private DifferentialDriveOdometry m_drive;

	private PigeonIMU.GeneralStatus status;
	private PigeonIMU imu;

	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
	private double[] ypr = new double[3];

	public DriveBase() {
		super();

		status = new PigeonIMU.GeneralStatus();
		imu = new PigeonIMU(Constants.PIGEON_IMU_ID);

		CANSparkMax leftLead = new CANSparkMax(Constants.LEFT_LEAD, MotorType.kBrushless);
		CANSparkMax leftFollow = new CANSparkMax(Constants.LEFT_F, MotorType.kBrushless);

		CANSparkMax rightLead = new CANSparkMax(Constants.RIGHT_LEAD, MotorType.kBrushless);
		CANSparkMax rightFollow = new CANSparkMax(Constants.RIGHT_F, MotorType.kBrushless);

		// Note that one pod must be inverted, since the gearbox assemblies are
		// rotationally symmetrical
		leftPod = new DrivePodSpark("Left", leftLead, leftFollow, false);
		rightPod = new DrivePodSpark("Right", rightLead, rightFollow, true);

		m_drive = new DifferentialDriveOdometry(getYaw());
	}

	public void init() {
		imu.getResetCount();
		leftPod.resetWheelPositions();
		rightPod.resetWheelPositions();
	}

	// TODO: Add support for various other drive types for future seasons
	// region Drive functions
	public void drive(double xSpeed, double rot) {
		var wheelSpeeds = Constants.DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}
	// endregion

	// region Differential Drive Helper Functions
	public void setDrivePodVoltages(double leftVolts, double rightVolts) {
		leftPod.setVoltage(leftVolts);
		rightPod.setVoltage(rightVolts);
	}

	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		final double leftFeedForward = m_feedforward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedForward = m_feedforward.calculate(speeds.rightMetersPerSecond);

		leftPod.setVoltage(speeds.leftMetersPerSecond + leftFeedForward);
		rightPod.setVoltage(speeds.rightMetersPerSecond + rightFeedForward);
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
	}

	public double[] getWheelPositions() {
		double[] positions = new double[2];
		positions[0] = leftPod.getWheelPositions();
		positions[1] = rightPod.getWheelPositions();
		return positions;
	}
	// endregion

	// region IMU Helper Functions
	public Rotation2d getYaw() {
		imu.getYawPitchRoll(ypr);
		Rotation2d heading = new Rotation2d(Math.toRadians(ypr[0]));
		return heading;
	}

	public void resetGyro() {
		imu.setYaw(0);
	}

	public void resetOdometry(Pose2d pose) {
		leftPod.resetWheelPositions();
		rightPod.resetWheelPositions();
		m_drive.resetPosition(pose, getYaw());
	}

	public Pose2d getPose() {
		return m_drive.getPoseMeters();
	}
	// endregion

	/**
	 * Turn dynamic braking on or off
	 * 
	 * @param isEnabled true to brake, false to freewheel
	 */
	public void brake(boolean isEnabled) {
		leftPod.enableBrakeMode(isEnabled);
		rightPod.enableBrakeMode(isEnabled);
	}

	/**
	 * Get the instantaneous speed of the left side drive pod, in feet per second
	 * 
	 * @return instantaneous speed of the left side drive pod, in feet per second
	 */
	public double getLeftSpeed() {
		return leftPod.getEncoderVelocityFeetPerSecondSansGear() / Constants.LOW_GEAR_RATIO;
	}

	/**
	 * Get the instantaneous speed of the right side drive pod, in feet per second
	 * 
	 * @return instantaneous speed of the right side drive pod, in feet per second
	 */
	public double getRightSpeed() {
		return rightPod.getEncoderVelocityFeetPerSecondSansGear() / Constants.LOW_GEAR_RATIO;
	}

	/**
	 * Apply position control PID values
	 */
	public void applyPositionPidConsts() {
		leftPod.applyPositionPidConsts();
		rightPod.applyPositionPidConsts();
	}

	/**
	 * Apply set point for position control
	 */
	public void travleDistance(double rotations) {
		leftPod.travleDistance(rotations);
		rightPod.travleDistance(rotations);
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		applyPositionPidConsts();

		// Update odometry distance traveled
		m_drive.update(getYaw(), leftPod.getWheelPositions(), rightPod.getWheelPositions());

		// Display Data
		SmartDashboard.putNumber("Left velocity (ftps)", getLeftSpeed());
		SmartDashboard.putNumber("Right velocity (ftps)", getRightSpeed());
	}

	// This method will be called once per scheduler run during simulation
	@Override
	public void simulationPeriodic() {

	}
}
