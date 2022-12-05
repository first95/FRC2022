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
    public class Units {
        public static final double METERS_PER_INCH = 0.0254;
    }
    public class Drivebase {
        // Module locations, in meters, as distances to the center of the robot.
        // Positive x is torwards the robot front, and +y is torwards robot left.
        public static final double FRONT_LEFT_X = 0;
        public static final double FRONT_LEFT_Y = 0;
        public static final double FRONT_RIGHT_X = 0;
        public static final double FRONT_RIGHT_Y = 0;
        public static final double BACK_LEFT_X = 0;
        public static final double BACK_LEFT_Y = 0;
        public static final double BACK_RIGHT_X = 0;
        public static final double BACK_RIGHT_Y = 0;

        // Module PIDF gains
        public static final double MODULE_KP = 1;
        public static final double MODULE_KI = 0;
        public static final double MODULE_KD = 0;
        public static final double MODULE_IZ = 0;
        public static final double MODULE_KF = 0;

        // Encoder conversion values.  Drive converts motor rotations to linear wheel distance
        // and steering converts motor rotations to module azimuth
        public static final double METERS_PER_MOTOR_ROTATION = (4 * Math.PI * Units.METERS_PER_INCH) / 6.75;
            // Calculation: 4in diameter wheels * pi [circumfrence] * (meters/inch) / gear ratio
        public static final double DEGREES_PER_STEERING_ROTATION = 360 / 12.8;
            // degrees per rotation / gear ratio between module and motor
        
        // CAN IDs
        public static final int DRIVE_FL = 0;
        public static final int DRIVE_FR = 0;
        public static final int DRIVE_BL = 0;
        public static final int DRIVE_BR = 0;

        public static final int STEER_FL = 0;
        public static final int STEER_FR = 0;
        public static final int STEER_BL = 0;
        public static final int STEER_BR = 0;

        public static final int CANCODER_FL = 0; 
        public static final int CANCODER_FR = 0; 
        public static final int CANCODER_BL = 0; 
        public static final int CANCODER_BR = 0; 
    }
}
