/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
  public RobotMap() {
    configureLEDMotorControllers();
    configureDriveMotors();
  }

  /* LED Subsystem */
  public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

  void configureLEDMotorControllers() {
        
  }

  void configureDriveMotors() {
    // config factory default
  }
}
