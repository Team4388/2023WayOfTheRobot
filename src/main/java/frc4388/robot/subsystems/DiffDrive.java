/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.DriveConstants;
import frc4388.utility.RobotGyro;
import frc4388.utility.RobotTime;

/**
 * Add your docs here.
 */
public class DiffDrive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private RobotTime m_robotTime = RobotTime.getInstance();

  private WPI_TalonFX m_leftFrontMotor;
  private WPI_TalonFX m_rightFrontMotor;
  private WPI_TalonFX m_leftBackMotor;
  private WPI_TalonFX m_rightBackMotor;
  private DifferentialDrive m_driveTrain;
  private RobotGyro m_gyro;

  /**
   * Add your docs here.
   */
  public DiffDrive(WPI_TalonFX leftFrontMotor, WPI_TalonFX rightFrontMotor, WPI_TalonFX leftBackMotor,
      WPI_TalonFX rightBackMotor, DifferentialDrive driveTrain, RobotGyro gyro) {

    m_leftFrontMotor = leftFrontMotor;
    m_rightFrontMotor = rightFrontMotor;
    m_leftBackMotor = leftBackMotor;
    m_rightBackMotor = rightBackMotor;
    m_driveTrain = driveTrain;
    m_gyro = gyro;
  }

  @Override
  public void periodic() {
    m_gyro.updatePigeonDeltas();

    if (m_robotTime.m_frameNumber % DriveConstants.SMARTDASHBOARD_UPDATE_FRAME == 0) {
      updateSmartDashboard();
    }
  }

  /**
   * Add your docs here.
   */
  public void driveWithInput(double move, double steer) {
    m_driveTrain.arcadeDrive(move, steer);
  }

  /**
   * Add your docs here.
   */
  public void tankDriveWithInput(double leftMove, double rightMove) {
    m_leftFrontMotor.set(leftMove);
    m_rightFrontMotor.set(rightMove);
  }

  /**
   * Add your docs here.
   */
  private void updateSmartDashboard() {

    // Examples of the functionality of RobotGyro
    SmartDashboard.putBoolean("Is Gyro a Pigeon?", m_gyro.m_isGyroAPigeon);
    SmartDashboard.putNumber("Turn Rate", m_gyro.getRate());
    SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
    SmartDashboard.putData(m_gyro);
  }
}
