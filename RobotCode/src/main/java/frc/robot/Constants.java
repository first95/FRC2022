// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	    // How much the joystick must move before anything will happen
		public static double joystickDeadbandV = 0.07;
		public static double joystickDeadbandH = 0.05;
		
		// PCM Device ID
		public static final int PCM_NUM = 0;

		// Ground Pick-up IDs
		public static final int GROUND_PICK_UP_TALON_ID = 1;
		public static final int GROUND_PICK_UP_SOLENOID_ID = 1;

		// Singulator IDs
		public static final int INNER_SINGULATOR_TALON_ID = 3;

		// Singulator intake IDs
		public static final int SINGULATOR_INTAKE_TALON_ID = 2;

		// Ring Light controller IDs
		public static final int TARGET_CAM_GREEN_RINGLIGHT_TALON_ID = 4;

		// Shooter IDs
		public static final int LEADER_SHOOT = 12;
		public static final int FOLLOWER_SHOOT = 13;

		// Speed Shifter Values
		public static final double SPEED_TO_SHIFT_UP = 5.0; // ft per sec
		public static final double SPEED_TO_SHIFT_DOWN = 3.0; // ft per sec

		// Used with Talons
		public static final int PID_IDX = 0; // The Talons support up to 2 PID loops, with indexes 0 and 1.  We only use 0.
		public static final int CAN_TIMEOUT_MS = 10; // The amount of time to wait for the CAN transaction to finish
		public static final int CAN_ORDINAL_SLOT0 = 0;
		
		// Indices for solenoids
		public static final int SHIFTER_SOLENOID_NUM  = 0;
		public static final int CLIMBER_SOLENOID_NUM = 2;
		public static final int SHOOTER_HOOD_SOLENOID_ID = 3;
		
		// Drive base
		public static final int SUCKER = 5;
		// Indices for SparkMaxes
		public static final int LEFT_LEAD = 7;
		public static final int LEFT_F = 6;
		// public static final int LEFT_F2 = 12;
		public static final int RIGHT_LEAD = 9;
		public static final int RIGHT_F = 8;
		// public static final int RIGHT_F2 = 22;
		public static final int INDEXER_BELT_MOTOR_ID = 10;

		
		// Drivebase constants
		public static final double DRIVE_WHEEL_DIAMETER_IN = 6;
		public static final double LOW_GEAR_RATIO = 20.83;
		public static final double HIGH_GEAR_RATIO = 9.17;

		// Digital I/O pin names
		public static final int SINGULATOR_SENSOR = 0;
		public static final int INDEXER_ENTRANCE_SENSOR = 1;
		public static final int INDEXER_POWERCELL_LOADED_SENSOR = 2;
		public static final int SHOOTER_LOADED_SENSOR = 3;

		//For vision aiming
		public static final double VISION_CAM_FOV_Y_DEG = 49.7;
		public static final double VISION_CAM_FOV_X_DEG = 59.6;
		public static final double VISION_CAM_Y_PIXELS = 240;
		public static final double DEGREES_PER_PIXEL = VISION_CAM_FOV_Y_DEG / VISION_CAM_Y_PIXELS;

		public static final double TARGET_TALLNESS_INCHES = 17;
		public static final double CAM_HEIGHT_INCHES = 41.25;
		public static final double CAM_TILT_DEGREES = 31.93;
		public static final double TARGET_HEIGHT_INCHES = 98.25;
		public static final double HEIGHT_DIFFERENCE = TARGET_HEIGHT_INCHES - CAM_HEIGHT_INCHES;

		public static final double VISION_HEADING_TOLERANCE_DEG = 1;
		public static final double VISION_RANGE_TOLERANCE_INCH = 3;

		public static final double VISION_HEADING_MAX_SPEED_PERCENT = 0.75;
		public static final double VISION_HEADING_MIN_SPEED_PERCENT = 0.05;
		public static final double VISION_RANGE_MAX_SPEED_PERCENT = 1;
		public static final double VISION_RANGE_MIN_SPEED_PERCENT = 0.05;

		public static final double VISION_RANGE_A_INCH = 81;
		public static final double VISION_RANGE_B_INCH = 141;
		public static final double VISION_RANGE_C_INCH = 186;
		public static final double VISION_RANGE_D_INCH = 246;

		//For Shooting
		public static final double RPM_TO_SHOOTER_POWER_CONVERSION = 0.000168422; //Measured and calculated on 2021-02-19 with single-angle shooter
		public static final double SHOOTER_KP = 2.4; //Calculated via Zeigler-Nichols on 2021-08-12 with normal shooter
		public static final double SHOOTER_KI = 0.164759;
		public static final double SHOOTER_KD = 8.74;
}
