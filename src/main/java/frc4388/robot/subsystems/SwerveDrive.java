// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;


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
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.RobotGyro;

public class SwerveDrive extends SubsystemBase {
  
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SwerveModule[] modules;

  private Translation2d leftFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d leftBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);

  private RobotGyro gyro;

  private SwerveDriveOdometry odometry;

  public double speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW; // * slow by default
  public Rotation2d rotTarget = new Rotation2d();
  public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack, RobotGyro gyro) {
    this.leftFront = leftFront;
    this.rightFront = rightFront;
    this.leftBack = leftBack;
    this.rightBack = rightBack;
    this.gyro = gyro;

    this.odometry = new SwerveDriveOdometry(
      kinematics, 
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        leftFront.getPosition(),
        rightFront.getPosition(),
        leftBack.getPosition(),
        rightBack.getPosition()
      }
    );

    this.modules = new SwerveModule[] {this.leftFront, this.rightFront, this.leftBack, this.rightBack};
  }


  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    if (fieldRelative) {
      if (rightStick.getNorm() > 0.1) {
        rotTarget = new Rotation2d(rightStick.getX(), -rightStick.getY()).minus(new Rotation2d(0, 1));
      }

      double rot = rotTarget.minus(gyro.getRotation2d()).getRadians();

      // Use the left joystick to set speed. Apply a quadratic curve and the set max speed.
      Translation2d speed = leftStick.times(leftStick.getNorm() * speedAdjust);

      // Convert field-relative speeds to robot-relative speeds.
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1 * speed.getX(), speed.getY(), rot * SwerveDriveConstants.ROTATION_SPEED, gyro.getRotation2d().times(-1));

    } else {
      // Create robot-relative speeds.
      chassisSpeeds = new ChassisSpeeds(-1 * leftStick.getX(), leftStick.getY(), rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED);
    }
    setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state, false);
    }
  }

  public void stopModules() {
    leftFront.stop();
    rightFront.stop();
    leftBack.stop();
    rightBack.stop();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
    setOdometry(getOdometry());
    rotTarget = new Rotation2d(0);
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
   * *NOTE: If you reset your gyro, this method MUST be called with the new gyro angle and wheel encoder positions.
   */
  public void resetOdometry() {
    setOdometry(new Pose2d());
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    // SmartDashboard.putNumberArray("Odometry", new double[] {getOdometry().getX(), getOdometry().getY(), getOdometry().getRotation().getDegrees()});
    SmartDashboard.putNumber("Odo X", getOdometry().getX());
    SmartDashboard.putNumber("Odo Y", getOdometry().getY());
    SmartDashboard.putNumber("Odo Theta", getOdometry().getRotation().getDegrees());
    
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  }

  /**
   * Shifts gear from high to low, or vice versa.
   * @param shift true to shift to high, false to shift to low
   */
  public void highSpeed(boolean shift) {
    this.speedAdjust = shift ? SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST : SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  }

}
