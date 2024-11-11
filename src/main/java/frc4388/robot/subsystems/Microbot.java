// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Microbot extends SubsystemBase {
  /** Creates a new Microbot. */

  TalonSRX frontLeft;
  TalonSRX backLeft;
  TalonSRX frontRight;
  TalonSRX backRight;

  public Microbot(TalonSRX frontLeft, TalonSRX backLeft, TalonSRX frontRight, TalonSRX backRight) {
    this.frontLeft = frontLeft;
    this.backLeft = backLeft;
    this.frontRight = backRight;
    this.backRight = backRight;
  }

  public void drive(double leftOutput, double rightOutput) {
    frontLeft.set(ControlMode.PercentOutput, leftOutput);
    backLeft.set(ControlMode.PercentOutput, leftOutput);
    frontRight.set(ControlMode.PercentOutput, rightOutput);
    backRight.set(ControlMode.PercentOutput, rightOutput);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
