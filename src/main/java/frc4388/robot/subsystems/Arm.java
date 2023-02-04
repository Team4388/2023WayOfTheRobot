// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonFX pivot;
  private WPI_TalonFX tele;
  
  

  public Arm(WPI_TalonFX pivot, WPI_TalonFX tele) {
    this.pivot = pivot;
    this.tele = tele;
    
    //Limit switches
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    tele.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    tele.configForwardSoftLimitThreshold(Constants.ArmConstants.TELE_FORWARD_SOFT_LIMIT);
  }

  public void runPivot(double output) {
    pivot.set(output);
  }

  public void runTele(double output) {
    tele.set(output);
  }

  public void runPivotAndTele(double pOutput, double tOutput) {
    pivot.set(pOutput);
    tele.set(tOutput);
  }

  public double getPivotPos() {
    return pivot.getSelectedSensorPosition();
  }

  public double getTelePos() {
    return tele.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if (tele.isRevLimitSwitchClosed() == 1) {
      tele.setSelectedSensorPosition(0);
    }
  }
}
