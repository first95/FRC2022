// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public enum ClimberStep{START, WINCH_OUT, PISTON_UP}

	public static class ClimberEnums {
		public static final int START = 0;
		public static final int WINCH_OUT = 1;
		public static final int PISTON_UP = 2;
	}

	/** Useful universal constants */
	public static class Universal {
		public static final double METERS_PER_INCH = 0.0254;
	}

	public static class Drivebase {
		// IDs
		public static final int LEFT_LEAD = 4;
		public static final int LEFT_F1 = 3;
		public static final int LEFT_F2 = 2;
		public static final int RIGHT_LEAD = 7;
		public static final int RIGHT_F1 = 6;
		public static final int RIGHT_F2 = 5;
		public static final int PIGEON_IMU_ID = 30;
		public static final int BRAKE_SOLENOID_ID = 14;

		// Drivebase Properties
		public static final double DRIVE_WHEEL_DIAMETER_IN = 4;
		public static final double LOW_GEAR_RATIO = 7;
		public static final double METERS_PER_ROTATION = (Math.PI * DRIVE_WHEEL_DIAMETER_IN * Universal.METERS_PER_INCH)
				/ (LOW_GEAR_RATIO);
		// For Pathfinding
		public static final double KS = 0.14053; // Calculated on 2022-02-12
		public static final double KV = 2.6647;
		public static final double KA = 0.34444;
		public static final double KP = 3.3956;
		public static final double EMPIRICAL_TRACKWIDTH_M = 1.101; // Calculated on 2022-02-14
		public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
				EMPIRICAL_TRACKWIDTH_M);
		public static final double MAX_SPEED_MPS = 3;
		public static final double MAX_ACCELERATION_MPSPS = 3;
		public static final double RAMSETE_B = 2;
		public static final double RAMSETE_ZETA = 0.7;
	}

	public static class Vision {
		// Limelight 2 camera properties
		public static final double CAM_FOV_Y_DEG = 49.7;
		public static final double CAM_FOV_X_DEG = 59.6;
		public static final double CAM_Y_PIXELS = 240;
		public static final double DEGREES_PER_PIXEL = CAM_FOV_Y_DEG / CAM_Y_PIXELS;

		// Target and limeligt mounting properties
		public static final double TARGET_WIDTH_INCHES = 5;
		public static final double CAM_HEIGHT_INCHES = 31;
		public static final double CAM_TILT_DEGREES = 47;
		public static final double TARGET_HEIGHT_INCHES = 102.5;
		public static final double HEIGHT_DIFFERENCE = TARGET_HEIGHT_INCHES - CAM_HEIGHT_INCHES;

		// Aiming tolerences, +/- setpoint
		public static final double HEADING_TOLERANCE_DEG = 5;
		public static final double RANGE_TOLERANCE_INCH = 12;

		// Max and min speeds
		public static final double HEADING_MAX_SPEED_PERCENT = 0.75;
		public static final double HEADING_MIN_SPEED_PERCENT = 0.05;
		public static final double RANGE_MAX_SPEED_PERCENT = 1;
		public static final double RANGE_MIN_SPEED_PERCENT = 0.1;

		public static final double DESIRED_RANGE_INCH = 35;
	}

	public static class CargoHandling {
		// IDs
		// Collector
		public static final int COLLECTOR_LEAD = 10;
		public static final int COLLECTOR_FOLLOW = 11;
		public static final int COLLECTOR_PNEUMATICS_ID = 15;
		// Singulator
		public static final int SINGULATOR_LEAD = 12;
		public static final int SINGULATOR_FOLLOW = 18;
		// Indexer
		public static final int INDEXER_MOTOR = 13;
		// Shooter
		public static final int SHOOTER_LEAD = 14;
		public static final int SHOOTER_FOLLOW = 15;
		public static final int SHOOTER_ROLLER_LEAD = 50;
		public static final int SHOOTER_ROLLER_FOLLOW = 60;
		// Digital I/O pin names
		public static final int INDEXER_LOADED_SENSOR_ID = 0;
		public static final int SHOOTER_LOADED_SENSOR_ID = 1;

		public static enum CargoColor {
			NONE, RIGHT, WRONG
		}

		public static Color RED_CARGO = new Color(0.525, 0.0, 0.125); // Red and blue cargo, ignoring green value
		public static Color BLUE_CARGO = new Color(0.152, 0.0, 0.457);

		public static final double INDEXING_SPEED = 0.5;
		public static final double INDEXER_REVERSE = -0.5;
		public static final double SHOOTING_INDEXER_SPEED = 0.4;
		public static final double SHOOTER_SLOW_SPEED = 2700;
		public static final double COLLECTOR_REVERSE = -1;
		public static final double SHOOTING_HIGH_SPEED = 3200;
		public static final double SHOOTING_LOW_SPEED = 2000;
		public static final double SHOOTER_IDLE_SPEED = 2000;

		// For Shooting
		public static final double RPM_TO_SHOOTER_POWER_CONVERSION = 0.00017048; // Measured and calculated on 2022-02-12
		public static final double SHOOTER_KP = 0.7; // Calculated via Zeigler-Nichols on 2021-08-12 with normal shooter
		public static final double SHOOTER_KI = 0;
		public static final double SHOOTER_KD = 0;
		public static final double SHOOTER_SPEED_TOLERANCE = 50; // Below setpoint
	}

	public static class Climber_Properties {
		// IDs
		public static final int LEFT_LEAD = 9;
		public static final int RIGHT_LEAD = 8;
		public static final int CLIMBER_PNEUMATICS_ID = 13;

		public static final double MAX_CLIMBER_SPEED = 0.8;

		// PID
		public static final int kP = 1;
		public static final int kI = 0;

	}
}
