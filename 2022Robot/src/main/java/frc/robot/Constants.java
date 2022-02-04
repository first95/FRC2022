// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	/**Useful universal constants*/
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

		// Drivebase Properties
		public static final double DRIVE_WHEEL_DIAMETER_IN = 6;
		public static final double LOW_GEAR_RATIO = 20.83;
		public static final double HIGH_GEAR_RATIO = 9.17;
		public static final double METERS_PER_ROTATION = (Math.PI * DRIVE_WHEEL_DIAMETER_IN * Universal.METERS_PER_INCH) / (LOW_GEAR_RATIO);
		// For Pathfinding
		public static final double KS = 0.14162;	//Calculated on 2022-01-14
		public static final double KV = 5.3687;
		public static final double KA = 0.28516;
		public static final double KP = 2.0063;
		public static final double EMPIRICAL_TRACKWIDTH_M = 0.7288;  //Calculated on 2022-01-28, was 0.83748
		public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(EMPIRICAL_TRACKWIDTH_M);
		public static final double MAX_SPEED_MPS = 2;
		public static final double MAX_ACCELERATION_MPSPS = 4;
		public static final double RAMSETE_B = 2;
		public static final double RAMSETE_ZETA = 0.7;
	}

	public static class Vision {
		public static final double CAM_FOV_Y_DEG = 49.7;
		public static final double CAM_FOV_X_DEG = 59.6;
		public static final double CAM_Y_PIXELS = 240;
		public static final double DEGREES_PER_PIXEL = CAM_FOV_Y_DEG / CAM_Y_PIXELS;

		public static final double TARGET_TALLNESS_INCHES = 17;
		public static final double CAM_HEIGHT_INCHES = 41.25;
		public static final double CAM_TILT_DEGREES = 31.93;
		public static final double TARGET_HEIGHT_INCHES = 98.25;
		public static final double HEIGHT_DIFFERENCE = TARGET_HEIGHT_INCHES - CAM_HEIGHT_INCHES;

		public static final double HEADING_TOLERANCE_DEG = 1;
		public static final double RANGE_TOLERANCE_INCH = 3;

		public static final double HEADING_MAX_SPEED_PERCENT = 0.75;
		public static final double HEADING_MIN_SPEED_PERCENT = 0.05;
		public static final double RANGE_MAX_SPEED_PERCENT = 1;
		public static final double RANGE_MIN_SPEED_PERCENT = 0.05;

		public static final double RANGE_A_INCH = 81;
		public static final double RANGE_B_INCH = 141;
		public static final double RANGE_C_INCH = 186;
		public static final double RANGE_D_INCH = 246;
	}

	public static class CargoHandling {
		// IDs
		// Collector
		public static final int COLLECTOR_LEAD = 0;
		public static final int COLLECTOR_FOLLOW = 0;
		// Singulator
		public static final int SINGULATOR_LEAD = 0;
		public static final int SINGULATOR_FOLLOW = 0;
		// Indexer
		public static final int INDEXER_MOTOR = 0;
		// Shooter
		public static final int SHOOTER_LEAD = 0;
		public static final int SHOOTER_FOLLOW = 0;
		public static final int SHOOTER_ROLLER_LEAD = 0;
		public static final int SHOOTER_ROLLER_FOLLOW = 0;
		// Digital I/O pin names
		public static final int INDEXER_LOADED_SENSOR_ID = 0;
		public static final int SHOOTER_LOADED_SENSOR_ID = 0;

		public static enum CargoColor {
			NONE, RIGHT, WRONG
		}

		public static final double INDEXING_SPEED = 0.5;
		public static final double INDEXER_REVERSE = -0.5;
		public static final double SHOOTING_INDEXER_SPEED = 0.8;
		public static final double SHOOTER_SLOW_SPEED = 100;
		public static final double COLLECTOR_REVERSE = -1;

		// For Shooting
		public static final double RPM_TO_SHOOTER_POWER_CONVERSION = 0.000168422; //Measured and calculated on 2021-02-19 with single-angle shooter
		public static final double SHOOTER_KP = 2.4; //Calculated via Zeigler-Nichols on 2021-08-12 with normal shooter
		public static final double SHOOTER_KI = 0.164759;
		public static final double SHOOTER_KD = 8.74;		
	}
}
