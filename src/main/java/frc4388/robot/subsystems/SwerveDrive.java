// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.RobotGyro;

public class SwerveDrive extends SubsystemBase {
  
  public SwerveModule leftFront;
  public SwerveModule rightFront;
  public SwerveModule leftBack;
  public SwerveModule rightBack;

  private SwerveModule[] modules;

  // private RobotGyro gyro3

  private Translation2d leftFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d leftBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation); 

  // private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
  //   kinematics, 
  //   gyro.getRotation2d(),
  //   new SwerveModulePosition[] {
  //     leftFront.getPosition(),
  //     rightFront.getPosition(),
  //     leftBack.getPosition(),
  //     rightBack.getPosition()
  //   }
  // );

  public double speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW; // * slow by default

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack) {//, RobotGyro gyro) {
    this.leftFront = leftFront;
    this.rightFront = rightFront;
    this.leftBack = leftBack;
    this.rightBack = rightBack;

    this.modules = new SwerveModule[] {this.leftFront, this.rightFront, this.leftBack, this.rightBack};
  }

  // public void driveWithInput(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
  //   Translation2d speed = new Translation2d(-xSpeed, ySpeed);
  //   double mag = speed.getNorm();
  //   speed = speed.times(mag * speedAdjust);
    
    // double xSpeedMetersPerSecond = -speed.getX(); //SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;
  //   double ySpeedMetersPerSecond = speed.getY(); //SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;
  //   // SwerveModuleState[] states = kinematics.toSwerveModuleStates(
  //   //   fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot, gyro.getRotation2d()) 
  //   //                 : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot)
  //   //);

  //   SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rot * SwerveDriveConstants.ROTATION_SPEED));

  //   setModuleStates(states);
  // }

  // public void driveWithInput(double leftX, double leftY, double rightX, double rightY, boolean fieldRelative) {
  //   // ignoreAngles = leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0;
  //   Translation2d speed = new Translation2d(-leftX, leftY);
  //   speed = speed.times(speed.getNorm() * speedAdjust);
  //   // if (Math.abs(rightX) > SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND || Math.abs(rightY) > SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
  //   //   rotTarget = new Rotation2d(rightX, -rightY).minus(new Rotation2d(0, 1));
  //   // double rot = rotTarget.minus(gyro.getRotation2d()).getRadians();
  //   double xSpeedMetersPerSecond = -speed.getX();
  //   double ySpeedMetersPerSecond = speed.getY();
  //   // chassisSpeeds = fieldRelative
  //   //     ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
  //   //         rot * SwerveDriveConstants.ROTATION_SPEED, m_gyro.getRotation2d())
  //   //     : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rightX * SwerveDriveConstants.ROTATION_SPEED);

  //   ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rightX * SwerveDriveConstants.ROTATION_SPEED);
    
  //   SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
  //   setModuleStates(states);
  // }

  // ! experimental WPILib swerve drive example
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // SwerveModuleState[] states = kinematics.toSwerveModuleStates(
    //   fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) 
    //                 : new ChassisSpeeds(xSpeed, ySpeed, rot)
    // );
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot * SwerveDriveConstants.ROTATION_SPEED));
    setModuleStates(states);
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.metersToFeet(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state);
    }
  }

  /**
   * Updates the odometry of the SwerveDrive.
   */
  // public void updateOdometry() {
  //   odometry.update(
  //     gyro.getRotation2d(), 
  //     new SwerveModulePosition[] {
  //       leftFront.getPosition(),
  //       rightFront.getPosition(),
  //       leftBack.getPosition(),
  //       rightBack.getPosition()
  //     }
  //   );
  // }

  /**
   * Gets the odometry of the SwerveDrive.
   * @return The odometry of the SwerveDrive as a Pose2d object (xMeters, yMeters, theta).
   */
  // public Pose2d getOdometry() {
  //   return odometry.getPoseMeters();
  // }

  /**
   * Sets the odometry of the SwerveDrive.
   * @param pose Pose to set the odometry to.
   */
  // public void setOdometry(Pose2d pose) {
  //   odometry.resetPosition(
  //     gyro.getRotation2d(), 
  //     new SwerveModulePosition[] {
  //       leftFront.getPosition(),
  //       rightFront.getPosition(),
  //       leftBack.getPosition(),
  //       rightBack.getPosition()
  //     },
  //     pose
  //   );
  // }

  /**
   * Resets the odometry of the SwerveDrive to 0.
   * *NOTE: If you reset your gyroscope or wheel encoders, this method MUST be called with the new gyro angle and wheel encoder positions.
   */
  // public void resetOdometry() {
  //   odometry.resetPosition(
  //     gyro.getRotation2d(), 
  //     new SwerveModulePosition[] {
  //       leftFront.getPosition(),
  //       rightFront.getPosition(),
  //       leftBack.getPosition(),
  //       rightBack.getPosition()
  //     },
  //     new Pose2d()
  //   );
  // }

  public void runAllDriveMotors(double output) {
    this.leftFront.driveMotor.set(output);
    this.rightFront.driveMotor.set(output);
    this.leftBack.driveMotor.set(output);
    this.rightBack.driveMotor.set(output);
  }

  public void runAllSteerMotors(double output) {
    this.leftFront.angleMotor.set(output);
    this.rightFront.angleMotor.set(output);
    this.leftBack.angleMotor.set(output);
    this.rightBack.angleMotor.set(output);
  }

  public void rotateCANCodersToAngle(double angle) {
    for (SwerveModule module : this.modules) {
      module.rotateToAngle(angle);
    }
  }

  public void resetCANCoders(double position) {
    for (SwerveModule module : this.modules) {
      module.reset(position);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateOdometry();
    SmartDashboard.putNumber("LeftFront CC", this.modules[0].getAngle().getDegrees());
    SmartDashboard.putNumber("RightFront CC", this.modules[1].getAngle().getDegrees());
    SmartDashboard.putNumber("LeftBack CC", this.modules[2].getAngle().getDegrees());
    SmartDashboard.putNumber("RightBack CC", this.modules[3].getAngle().getDegrees());
  }

  /**
   * Shifts gear from high to low, or vice versa.
   * @param shift true to shift to high, false to shift to low
   */
  public void highSpeed(boolean shift) {
    this.speedAdjust = shift ? SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST : SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  }

}
