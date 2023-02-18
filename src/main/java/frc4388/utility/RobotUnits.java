// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;


/** Aarav's good units class (better than WPILib)
 * @author Aarav Shah */

public final class RobotUnits {
  // variables
  private static boolean isSwerve;

  // constants
  private static final double INCHES_PER_FOOT = 12.0;
  private static final double METERS_PER_INCH = 0.0254;
  private static final double SECONDS_PER_MINUTE = 60;
  private static final double MILLISECONDS_PER_SECONDS = 1000;

  private static final int FALCON_TICKS_PER_MOTOR_REV = 2048;
  private static double WHEEL_DIAMETER_INCHES;
  private static double MOTOR_REV_PER_WHEEL_REV;
  private static double MOTOR_REV_PER_STEER_REV;
  


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
  }

  private static RobotUnits instance = null;

  public static RobotUnits getInstance() {
    if (instance == null) {
      instance = new RobotUnits(0, new double[] {0.0, 0.0});
    }
    return instance;
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
  public static double falconTicksToMotorRotations(final double ticks) {return ticks / FALCON_TICKS_PER_MOTOR_REV;}

  public static double falconMotorRotationsToTicks(final double rotations) {return rotations * FALCON_TICKS_PER_MOTOR_REV;}

  // TODO: Implement later

  // distance conversions
  public static double metersToInches(final double meters) {return meters / METERS_PER_INCH;}

  public static double inchesToMeters(final double inches) {return inches * METERS_PER_INCH;}

  public static double metersToFeet(final double meters) {return metersToInches(meters) / INCHES_PER_FOOT;}

  public static double feetToMeters(final double feet) {return inchesToMeters(feet * INCHES_PER_FOOT);}
}