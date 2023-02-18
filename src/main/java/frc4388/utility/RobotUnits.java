// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;


/** Aarav's good units class (better than WPILib)
 * @author Aarav Shah */

public final class RobotUnits {
  // constants
  private static final double INCHES_PER_FOOT = 12.0;
  private static final double METERS_PER_INCH = 0.0254;
  private static final double SECONDS_PER_MINUTE = 60;
  private static final double MILLISECONDS_PER_SECONDS = 1000;

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
  public static double falconTicksToRotations(final double ticks) {return ticks / 2048;}

  public static double falconRotationsToTicks(final double rotations) {return rotations * 2048;}

  // distance conversions
  public static double metersToInches(final double meters) {return meters / METERS_PER_INCH;}

  public static double inchesToMeters(final double inches) {return inches * METERS_PER_INCH;}

  public static double metersToFeet(final double meters) {return metersToInches(meters) / INCHES_PER_FOOT;}

  public static double feetToMeters(final double feet) {return inchesToMeters(feet * INCHES_PER_FOOT);}
}