// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonFX pivot;
  private WPI_TalonFX tele;

  public Arm(WPI_TalonFX pivot, WPI_TalonFX tele) {
    this.pivot = pivot;
    this.tele = tele;
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
    // This method will be called once per scheduler run
  }
}
