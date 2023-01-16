// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.RobotGyro;

public class SwerveDrive extends SubsystemBase {
  
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SwerveModule[] modules;

  private RobotGyro gyro;

  private Translation2d leftFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d leftBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation); 

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, 
    gyro.getRotation2d(),
    new SwerveModulePosition[] {
      leftFront.getPosition(),
      rightFront.getPosition(),
      leftBack.getPosition(),
      rightBack.getPosition()
    }
  );

  public double speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW; // * slow by default

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack, RobotGyro gyro) {
    this.leftFront = leftFront;
    this.rightFront = rightFront;
    this.leftBack = leftBack;
    this.rightBack = rightBack;

    this.modules = new SwerveModule[] {this.leftFront, this.rightFront, this.leftBack, this.rightBack};

    this.gyro = gyro;
  }

  public void driveWithInput(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedMetersPerSecond = xSpeed * SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;
    double ySpeedMetersPerSecond = ySpeed * SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot, gyro.getRotation2d()) 
                    : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Units.metersToFeet(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    setModuleStates(states);
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state);
    }
  }

  /**
   * Updates the odometry of the SwerveDrive.
   */
  public void updateOdometry() {
    odometry.update(
      gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      }
    );
  }

  /**
   * Gets the odometry of the SwerveDrive.
   * @return The odometry of the SwerveDrive as a Pose2d object (xMeters, yMeters, theta).
   */
  public Pose2d getOdometry() {
    return odometry.getPoseMeters();
  }

  /**
   * Sets the odometry of the SwerveDrive.
   * @param pose Pose to set the odometry to.
   */
  public void setOdometry(Pose2d pose) {
    odometry.resetPosition(
      gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      },
      pose
    );
  }

  /**
   * Resets the odometry of the SwerveDrive to 0.
   */
  public void resetOdometry() {
    odometry.resetPosition(
      gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      },
      new Pose2d()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Shifts gear from high to low, or vice versa.
   * @param shift true to shift to high, false to shift to low
   */
  public void highSpeed(boolean shift) {
    this.speedAdjust = shift ? SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST : SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  }

}
