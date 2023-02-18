// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;


/** Aarav's good units class (better than WPILib)
 * @author Aarav Shah */

public final class RobotUnits {
  // variables
  private static boolean isSwerve;

  // measurement constants
  private static final double INCHES_PER_FOOT = 12.0;
  private static final double METERS_PER_INCH = 0.0254;
  private static final double SECONDS_PER_MINUTE = 60;
  private static final double MILLISECONDS_PER_SECONDS = 1000;

  // robot constants
  private static final int TICKS_PER_MOTOR_REV = 2048;
  private static double WHEEL_DIAMETER_INCHES;
  private static double MOTOR_REV_PER_WHEEL_REV;
  private static double MOTOR_REV_PER_STEER_REV;
  private static double INCHES_PER_WHEEL_REV;
  private static double TICKS_PER_WHEEL_REV;

  // final conversions
  private static final double TICK_TIME_TO_SECONDS = 0.1;
  private static double TICKS_PER_INCH;

  
  // private constructor
  private RobotUnits(final double WHEEL_DIAMETER_INCHES, final double[] gearRatios) throws IllegalArgumentException {
    if (gearRatios.length == 1) {
      RobotUnits.isSwerve = false;

      RobotUnits.MOTOR_REV_PER_WHEEL_REV = gearRatios[0];

    } else if (gearRatios.length == 2) {
      RobotUnits.isSwerve = true;

      RobotUnits.MOTOR_REV_PER_WHEEL_REV = gearRatios[0];
      RobotUnits.MOTOR_REV_PER_STEER_REV = gearRatios[1];

    } else {
      throw new IllegalArgumentException();
    }

    RobotUnits.WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_INCHES;
    RobotUnits.initVar();
  }

  private static RobotUnits instance = null;

  public static RobotUnits getInstance(final double WHEEL_DIAMETER_INCHES, final double[] gearRatios) {
    if (instance == null) {
      instance = new RobotUnits(WHEEL_DIAMETER_INCHES, gearRatios);
    }
    return instance;
  }

  private static void initVar() {
    INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
    TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_WHEEL_REV;

    TICKS_PER_INCH = TICKS_PER_WHEEL_REV / INCHES_PER_WHEEL_REV;
  }

  // angle conversions
  public static double degreesToRadians(final double degrees) {return Math.toRadians(degrees);}

  public static double radiansToDegrees(final double radians) {return Math.toDegrees(radians);}

  public static double radiansToRotations(final double radians) {return radians / (Math.PI * 2);}

  public static double degreesToRotations(final double degrees) {return degrees / 360;}

  public static double rotationsToRadians(final double rotations) {return rotations * (Math.PI * 2);}

  public static double rotationsToDegrees(final double rotations) {return rotations * 360;}

  // angular velocity conversions
  public static double rotationsPerMinuteToRadiansPerSecond(final double rotationsPerMinute) {return rotationsPerMinute * Math.PI / (SECONDS_PER_MINUTE / 2);}

  public static double radiansPerSecondToRotationsPerMinute(final double radiansPerSecond) {return radiansPerSecond * (SECONDS_PER_MINUTE / 2) / Math.PI;}

  // time conversions
  public static double millisecondsToSeconds(final double milliseconds) {return milliseconds / MILLISECONDS_PER_SECONDS;}

  public static double secondsToMilliseconds(final double seconds) {return seconds * MILLISECONDS_PER_SECONDS;}

  // falcon conversions
  public static double falconTicksToMotorRotations(final double ticks) {return ticks / TICKS_PER_MOTOR_REV;}

  public static double falconMotorRotationsToTicks(final double rotations) {return rotations * TICKS_PER_MOTOR_REV;}
  
  public static double inchesToTicks(final double inches) {return inches * TICKS_PER_INCH;}
  
  public static double ticksToInches(final double ticks) {return ticks / TICKS_PER_INCH;}

  public static double secondsToTickTime(final double seconds) {return seconds * TICK_TIME_TO_SECONDS;}
  
  public static double tickTimeToSeconds(final double tickTime) {return tickTime / TICK_TIME_TO_SECONDS;}

  // distance conversions
  public static double metersToInches(final double meters) {return meters / METERS_PER_INCH;}

  public static double inchesToMeters(final double inches) {return inches * METERS_PER_INCH;}

  public static double metersToFeet(final double meters) {return metersToInches(meters) / INCHES_PER_FOOT;}

  public static double feetToMeters(final double feet) {return inchesToMeters(feet * INCHES_PER_FOOT);}
}