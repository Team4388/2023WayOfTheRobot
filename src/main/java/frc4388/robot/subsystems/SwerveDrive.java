// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

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

  // WPILib swerve drive example
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // SwerveModuleState[] states = kinematics.toSwerveModuleStates(
    //   fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getRotation2d().getDegrees())) 
    //                 : new ChassisSpeeds(xSpeed, ySpeed, rot)
    // );

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.3, 0, 0));

    // // SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot * SwerveDriveConstants.ROTATION_SPEED));
    setModuleStates(states);
    // modules[0].getDriveMotor().set(0.2);
    // modules[1].getDriveMotor().set(0.2);
    // modules[2].getDriveMotor().set(0.2);
    // modules[3].getDriveMotor().set(0.2);

    // modules[0].getAngleMotor().set(TalonFXControlMode.Position, 1017);
    // modules[1].getAngleMotor().set(TalonFXControlMode.Position, 509);
    // modules[2].getAngleMotor().set(TalonFXControlMode.Position, 683);
    // modules[3].getAngleMotor().set(TalonFXControlMode.Position, 1816);
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

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
    setOdometry(getOdometry());
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
    
    // SmartDashboard.putNumber("LF CC Angle", modules[0].getAngle().getDegrees());
    // SmartDashboard.putNumber("RF CC Angle", modules[1].getAngle().getDegrees());
    // SmartDashboard.putNumber("LB CC Angle", modules[2].getAngle().getDegrees());
    // SmartDashboard.putNumber("RB CC Angle", modules[3].getAngle().getDegrees());
    
    SmartDashboard.putNumber("LF Vel", modules[0].getDriveVel());
    SmartDashboard.putNumber("RF Vel", modules[1].getDriveVel());
    SmartDashboard.putNumber("LB Vel", modules[2].getDriveVel());
    SmartDashboard.putNumber("RB Vel", modules[3].getDriveVel());

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());

    SmartDashboard.putNumber("LF Pos", modules[0].getDrivePos());
    SmartDashboard.putNumber("RF Pos", modules[1].getDrivePos());
    SmartDashboard.putNumber("LB Pos", modules[2].getDrivePos());
    SmartDashboard.putNumber("RB Pos", modules[3].getDrivePos());
    
  }

  /**
   * Shifts gear from high to low, or vice versa.
   * @param shift true to shift to high, false to shift to low
   */
  public void highSpeed(boolean shift) {
    this.speedAdjust = shift ? SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST : SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  }

}
