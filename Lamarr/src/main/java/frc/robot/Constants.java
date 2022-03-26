// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

	public static class Auton {
		public static final int TWO_CARGO_REVERSED_GET = 0;
		public static final int FoB1_Backup = 1;
		public static final int FoB2_K1 = 2;
		public static final int FoB3_Get2 = 3;
		public static final int FoB4_Shoot2 = 4;

		public static final String [] trajectoryFiles = {
			"ReverseTwoCargo1",
			"FoB1_Backup",
			"FoB2_K1",
			"FoB3_Get2",
			"FoB4_Shoot2"
		};
	}

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
		public static final double KS = 0.13528; //calculated on 2022-03-1//0.14053; // Calculated on 2022-02-12
		public static final double KV = 2.7879; //2.6647;
		public static final double KA = 0.47654; //0.34444;
		public static final double KP = 3.7363; //3.3956;
		public static final double EMPIRICAL_TRACKWIDTH_M = 0.6; // Calculated on 2022-03-14 as 0.576948, tuned to 0.8
		public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
				EMPIRICAL_TRACKWIDTH_M);
		public static final double MAX_SPEED_MPS = 4.5;
		public static final double MAX_ACCELERATION_MPSPS = 2.5;
		public static final double RAMSETE_B = 2;
		public static final double RAMSETE_ZETA = 0.7;
	}

	public static class Vision {
		// Limelight 2 camera properties
		public static final double CAM_FOV_Y_DEG = 49.7;
		public static final double CAM_FOV_X_DEG = 59.6;
		public static final double CAM_Y_PIXELS = 240;
		public static final double DEGREES_PER_PIXEL = CAM_FOV_Y_DEG / CAM_Y_PIXELS;

		// Target and limelight mounting properties
		public static final double TARGET_WIDTH_INCHES = 5;
		public static final double CAM_HEIGHT_INCHES = 31;
		public static final double CAM_TILT_DEGREES = 32.5;
		public static final double TARGET_HEIGHT_INCHES = 102.5;
		public static final double HEIGHT_DIFFERENCE = TARGET_HEIGHT_INCHES - CAM_HEIGHT_INCHES;

		// Aiming tolerences, +/- setpoint
		public static final double HEADING_TOLERANCE_DEG = 1.0;
		public static final double RANGE_TOLERANCE_INCH = 15;

		// Max and min speeds
		public static final double HEADING_MAX_SPEED_PERCENT = 0.75;
		public static final double HEADING_MIN_SPEED_PERCENT = 0.05;
		public static final double RANGE_MAX_SPEED_PERCENT = 1;
		public static final double RANGE_MIN_SPEED_PERCENT = 0.1;

		// PID Gains
		public static final double HEADING_KP = 0.5; //0.3
		public static final double HEADING_KI = 0;
		public static final double HEADING_KD = 0;
		public static final double RANGE_KP = 0.25;
		public static final double RANGE_KI = 0;
		public static final double RANGE_KD = 0;

		public static final double DESIRED_RANGE_INCH = 115; // OLD DISTANCE: 115, 160 (3744/1874)
	}

	public static class CargoHandling {
		// IDs
		// Collector
		public static final int COLLECTOR_LEAD = 10;
		public static final int COLLECTOR_FOLLOW = 16;
		public static final int COLLECTOR_PNEUMATICS_ID = 15;
		// Singulator
		public static final int SINGULATOR_LEAD = 12; 
		public static final int SINGULATOR_FOLLOW = 18;
		// Indexer
		public static final int INDEXER_MOTOR = 13;
		// Shooter
		public static final int SHOOTER = 15;
		public static final int SHOOTER_ROLLER = 17;
		// Digital I/O pin names
		public static final int INDEXER_LOADED_SENSOR_ID = 0;
		public static final int SHOOTER_LOADED_SENSOR_ID = 1;

		public static enum CargoColor {
			NONE, RIGHT, WRONG
		}

		// Speeds
		public static final double INDEXING_SPEED = 0.5;
		public static final double INDEXER_REVERSE = 0.0;
		public static final double SHOOTING_INDEXER_SPEED = 0.7; // Testing indexing speeds (shooter debugging)
		public static final double SHOOTER_SLOW_SPEED = 1500;
		public static final double COLLECTOR_REVERSE = -1;
		public static final double SHOOTING_LOW_SPEED = 1500;
		public static final double SHOOTER_IDLE_SPEED = 1500;
		public static final double ROLLER_IDLE_SPEED = 3000;
		public static final double ROLLER_EJECT_SPEED = 3000;
		public static final double ROLLER_LOW_SPEED = 3000;
		public static final double MANUAL_SHOOTING_SPEED = 1800;

		public static final double SHOOTER_SPEED_M = 5; //4.7798;
		public static final double SHOOTER_SPEED_B = 1225; //1243.07;
		public static final double SHOOTER_RATIO = 2.3;

		// For Shooting
		public static final double RPM_TO_SHOOTER_POWER_CONVERSION = 0.00017048; // Measured and calculated on 2022-02-12
		public static final double SHOOTER_KP = 0.7;
		public static final double SHOOTER_SPEED_TOLERANCE = 25; // Below setpoint
		public static final double RPM_TO_ROLLER_POWER_CONVERSION = 0.00017022;
		public static final double ROLLER_KP = 0.6;
		public static final double ROLLER_SPEED_TOLERANCE = 100;

		// Only shoot when Previous Yaw +- YAW_THRESHOLD = Current YAW
		public static double YAW_THRESHOLD = 0.1;
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
		public static final double NEW_GEAR = 1;

	}
}
