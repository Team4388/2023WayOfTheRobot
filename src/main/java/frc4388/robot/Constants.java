/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public static final double MAX_ROT_SPEED        = 1.5;
    public static final double MIN_ROT_SPEED        = 0.8;
    public static       double ROTATION_SPEED       = MAX_ROT_SPEED;
    public static       double ROT_CORRECTION_SPEED = MIN_ROT_SPEED;

    public static final class IDs {
      public static final int LEFT_FRONT_WHEEL_ID = 2;
      public static final int LEFT_FRONT_STEER_ID = 3;
      public static final int LEFT_FRONT_ENCODER_ID = 10;

      public static final int RIGHT_FRONT_WHEEL_ID = 4;
      public static final int RIGHT_FRONT_STEER_ID = 5;
      public static final int RIGHT_FRONT_ENCODER_ID = 11;

      public static final int LEFT_BACK_WHEEL_ID = 6;
      public static final int LEFT_BACK_STEER_ID = 7;
      public static final int LEFT_BACK_ENCODER_ID = 12;
      
      public static final int RIGHT_BACK_WHEEL_ID = 8;
      public static final int RIGHT_BACK_STEER_ID = 9;
      public static final int RIGHT_BACK_ENCODER_ID = 13;
    }

    public static final class PIDConstants {
      public static final int SWERVE_SLOT_IDX = 0;
      public static final int SWERVE_PID_LOOP_IDX = 1;
      public static final Gains SWERVE_GAINS = new Gains(0.5, 0.0, 0.0, 0.0, 0, 1.0);
    }

    public static final class AutoConstants {
      public static final Gains X_CONTROLLER = new Gains(0.8, 0.0, 0.0);
      public static final Gains Y_CONTROLLER = new Gains(0.8, 0.0, 0.0);
      public static final Gains THETA_CONTROLLER = new Gains(-0.8, 0.0, 0.0);
      public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI/2, Math.PI/2); // TODO: tune

      public static final double PATH_MAX_VEL = 0.3; // TODO: find the actual value
      public static final double PATH_MAX_ACC = 0.3; // TODO: find the actual value
    }

    public static final class Conversions {
      public static final int CANCODER_TICKS_PER_ROTATION = 4096;

      public static final double JOYSTICK_TO_METERS_PER_SECOND_FAST = 4.8;
      public static final double JOYSTICK_TO_METERS_PER_SECOND_SLOW = 0.8;

      public static final double MOTOR_REV_PER_WHEEL_REV = 5.12;
      public static final double MOTOR_REV_PER_STEER_REV = 12.8;

      public static final double TICKS_PER_MOTOR_REV = 2048;
      public static final double WHEEL_DIAMETER_INCHES = 3.9;
      public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;

      public static final double WHEEL_REV_PER_MOTOR_REV = 1 / MOTOR_REV_PER_WHEEL_REV;
      public static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_WHEEL_REV;
      public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REV / INCHES_PER_WHEEL_REV;
      public static final double INCHES_PER_TICK = 1 / TICKS_PER_INCH;

      public static final double TICK_TIME_TO_SECONDS = 10;
      public static final double SECONDS_TO_TICK_TIME = 1 / TICK_TIME_TO_SECONDS;
    }

    public static final class Configurations {
      public static final double OPEN_LOOP_RAMP_RATE = 0.2; // TODO: find the actual value
      public static final double CLOSED_LOOP_RAMP_RATE = 0.2; // TODO: find the actual value
      public static final double NEUTRAL_DEADBAND = 0.04; // TODO: find the actual value
    }

    public static final double MAX_SPEED_FEET_PER_SECOND = 5; // TODO: find the actual value
    public static final double MAX_ANGULAR_SPEED_FEET_PER_SECOND = 2 * 2 * Math.PI; // TODO: find the actual value

    // dimensions
    public static final double WIDTH = 18.5;
    public static final double HEIGHT = 18.5;
    public static final double HALF_WIDTH = WIDTH / 2.d;
    public static final double HALF_HEIGHT = HEIGHT / 2.d;

    // misc
    public static final int TIMEOUT_MS = 30;
    public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
  }

  public static final class GyroConstants {
    public static final int ID = 14;
  }

  public static final class ArmConstants {
    public static final double MIN_ARM_LEN = 1;
    public static final double MAX_ARM_LEN = 2;
    public static final double ARM_HEIGHT  = 1;
    public static final double CURVE_POWER = 2;

    public static final double TELE_TICKS_PER_SECOND = (-5);
    
    public static final double PIVOT_FORWARD_SOFT_LIMIT = 100; // TODO: find actual value
    public static final double PIVOT_REVERSE_SOFT_LIMIT = 0; // TODO: find actual value
    
    public static final double TELE_FORWARD_SOFT_LIMIT = 100;
    public static final double TELE_REVERSE_SOFT_LIMIT = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double OFFSET = 0;
  }

  public static final class LEDConstants {
    // public static final int LED_SPARK_ID = 0;

    public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
  }

  public static final class OIConstants {
    public static final int XBOX_DRIVER_ID = 0;
    public static final int XBOX_OPERATOR_ID = 1;
    public static final double LEFT_AXIS_DEADBAND = 0.1;
    public static final double RIGHT_AXIS_DEADBAND = 0.6;
    public static final boolean SKEW_STICKS = true; // ! this might have to actually be false, merge conflicts are confusing
  }
}
