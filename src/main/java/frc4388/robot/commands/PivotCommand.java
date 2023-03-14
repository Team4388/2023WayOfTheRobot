// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import frc4388.robot.subsystems.Arm;

public class PivotCommand extends PelvicInflammatoryDisease {
  private final Arm     arm;
  private final double  target;
  
  /** Creates a new ArmCommand. */
  public PivotCommand(Arm arm, double target) {
    super(0.6, 0, 0, 0, 0);
    this.arm    = arm;
    this.target = target;
    addRequirements(arm);
  }

  @Override
  public double getError() {
    return (arm.getArmRotation() - target) / 360;
  }

  @Override
  public void runWithOutput(double output) {
    arm.setRotVel(output);
  }
}
