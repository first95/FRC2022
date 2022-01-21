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
    // Indices for SparkMaxes
	public static final int LEFT_LEAD = 7;
	public static final int LEFT_F = 6;
	public static final int RIGHT_LEAD = 9;
	public static final int RIGHT_F = 8;

	// Drivebase constants
	public static final double DRIVE_WHEEL_DIAMETER_IN = 6;
	public static final double METERS_PER_INCH = 0.0254;
	public static final double LOW_GEAR_RATIO = 20.83;
	public static final double HIGH_GEAR_RATIO = 9.17;
	public static final double METERS_PER_ROTATION = (Math.PI * DRIVE_WHEEL_DIAMETER_IN * METERS_PER_INCH) / (LOW_GEAR_RATIO);
	// For Pathfinding
	public static final double KS = 0.14162;	//Calculated on 2022-01-14
	public static final double KV = 5.3687;
	public static final double KA = 0.28516;
	public static final double KP = 2.0063;
	public static final double EMPIRICAL_TRACKWIDTH_M = 0.83748;
	public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(EMPIRICAL_TRACKWIDTH_M);
	public static final double MAX_SPEED_MPS = 2;
	public static final double MAX_ACCELERATION_MPSPS = 2;
	public static final double RAMSETE_B = 2;
	public static final double RAMSETE_ZETA = 0.7;

	//Pigeon ID
	public static final int PIGEON_IMU_ID = 30;
}
