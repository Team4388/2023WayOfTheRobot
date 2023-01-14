/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveDriveConstants {
        public static final class IDs {
            public static final int DRIVE_PIGEON_ID = -1; // TODO: find actual ID

            public static final int LEFT_FRONT_WHEEL_ID = -1; // TODO: find actual ID
            public static final int RIGHT_FRONT_WHEEL_ID = -1; // TODO: find actual ID
            public static final int LEFT_BACK_WHEEL_ID = -1; // TODO: find actual ID
            public static final int RIGHT_BACK_STEER_ID = -1; // TODO: find actual ID

            public static final int LEFT_FRONT_STEER_ID = -1; // TODO: find actual ID
            public static final int RIGHT_FRONT_STEER_ID = -1; // TODO: find actual ID
            public static final int LEFT_BACK_STEER_ID = -1; // TODO: find actual ID
            public static final int RIGHT_BACK_WHEEL_ID = -1; // TODO: find actual ID

            public static final int LEFT_FRONT_ENCODER_ID = -1; // TODO: find actual ID
            public static final int RIGHT_FRONT_ENCODER_ID = -1; // TODO: find actual ID
            public static final int LEFT_BACK_ENCODER_ID = -1; // TODO: find actual ID
            public static final int RIGHT_BACK_ENCODER_ID = -1; // TODO: find actual ID
        }

        public static final class PIDConstants {
            public static final int SWERVE_SLOT_IDX = 0;
            public static final int SWERVE_PID_LOOP_IDX = 1;
            public static final Gains SWERVE_GAINS = new Gains(1.0, 0.0, 0.0, 0.0, 0, 1.0);
        }

        public static final class Conversions {
            public static final int CANCODER_TICKS_PER_ROTATION = 4096;
            public static final double JOYSTICK_TO_METERS_PER_SECOND = 5.0;
        }

        public static final class Configurations {
            public static final double OPEN_LOOP_RAMP_RATE = 0.2; // TODO: find the actual value
            public static final double CLOSED_LOOP_RAMP_RATE = 0.2; // TODO: find the actual value
            public static final double NEUTRAL_DEADBAND = 0.04; // TODO: find the actual value

            public static final double LEFT_FRONT_ENCODER_OFFSET = -1.0; // TODO: find the actual value
            public static final double RIGHT_FRONT_ENCODER_OFFSET = -1.0; // TODO: find the actual value
            public static final double LEFT_BACK_ENCODER_OFFSET = -1.0; // TODO: find the actual value
            public static final double RIGHT_BACK_ENCODER_OFFSET = -1.0; // TODO: find the actual value
        }

        public static final double MAX_SPEED_FEET_PER_SECOND = -1; // TODO: find the actual value

        // dimensions
        public static final double WIDTH = -1; // TODO: find the actual value
        public static final double HEIGHT = -1; // TODO: find the actual value
        public static final double HALF_WIDTH = WIDTH / 2.d;
        public static final double HALF_HEIGHT = HEIGHT / 2.d;

        // misc
        public static final int TIMEOUT_MS = 30;
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    }
    
    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
    }
}
