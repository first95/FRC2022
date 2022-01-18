
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The DriveBase subsystem incorporates the sensors and actuators attached to
 * the robot's chassis. These include two 3-motor drive pods.
 */
public class DriveBase extends SubsystemBase {

	private CANSparkMax leftPod, leftfollow, rightPod, rightfollow;
	private RelativeEncoder leftEncoder, rightEncoder;
	private final DifferentialDriveOdometry m_odometry;

	private Solenoid shifter;

	private TalonSRX sucker;


	private double [] ypr = new double [3];
	private PigeonIMU.GeneralStatus status = new PigeonIMU.GeneralStatus();
	private PigeonIMU imu = new PigeonIMU(Constants.PIGEON_IMU_ID);


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

		leftEncoder = leftPod.getEncoder();
		rightEncoder = rightPod.getEncoder();
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);

		m_odometry = new DifferentialDriveOdometry(getYaw());



		// shifter = new Solenoid(Constants.SHIFTER_SOLENOID_NUM); // 2020 API Version
		shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHIFTER_SOLENOID_NUM);

		sucker = new TalonSRX(Constants.SUCKER);

		imu.setYaw(0);
		setGear(false);
	}

	public void init() {
		rightPod.setInverted(true);
	}
	
	public void driveWithJoysticks(){
		double x = RobotContainer.oi.getForwardAxis();
		double y = RobotContainer.oi.getTurnAxis();
		x = Math.pow(x, 3);
		y = Math.pow(y, 3);
		driveWithTankControls(x + y, x - y);
	}

	public void driveWithTankControls(double left, double right) {
		leftPod.set(left);
		rightPod.set(right);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		leftPod.setVoltage(leftVolts);
		rightPod.setVoltage(rightVolts);
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		double leftVelocity = (leftEncoder.getVelocity() * Constants.METERS_PER_ROTATION) / 60;
		double rightVelocity = (rightEncoder.getVelocity() * Constants.METERS_PER_ROTATION) / 60;
		return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
	}

	public double [] getWheelPositions() {
		double [] positions = new double [2];
		positions[0] = (leftEncoder.getPosition() * Constants.METERS_PER_ROTATION);
		positions[1] = (rightEncoder.getPosition() * Constants.METERS_PER_ROTATION);
		return positions;
	}

	public Rotation2d getYaw() {
		imu.getYawPitchRoll(ypr);
		Rotation2d heading = new Rotation2d(Math.toRadians(ypr[0]));
		return heading;
	}

	public void resetGyro() {
		imu.setYaw(0);
	}

	public void resetOdometry(Pose2d pose) {
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
		m_odometry.resetPosition(pose, getYaw());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
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
		imu.getGeneralStatus(status);
		m_odometry.update(getYaw(), getWheelPositions()[0], getWheelPositions()[1]);
		SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
		SmartDashboard.putNumber("LeftM", getWheelPositions()[0]);
		SmartDashboard.putNumber("RightM", getWheelPositions()[1]);
		SmartDashboard.putNumber("left", leftEncoder.getPosition());
		SmartDashboard.putNumber("right", rightEncoder.getPosition());
		SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());

	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
