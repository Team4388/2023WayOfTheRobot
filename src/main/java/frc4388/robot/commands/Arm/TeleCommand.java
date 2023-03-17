// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Arm;

import frc4388.robot.Constants.ArmConstants;
import frc4388.robot.commands.PelvicInflammatoryDisease;
import frc4388.robot.subsystems.Arm;

public class TeleCommand extends PelvicInflammatoryDisease {
  private final Arm    arm;
  private final double target;
  
  /** Creates a new ArmCommand. */
  public TeleCommand(Arm arm, double target) {
    super(0.6, 0, 0, 0, 0);
    this.arm    = arm;
    this.target = target;
    addRequirements(arm);
  }

  @Override
  public double getError() {
    return (arm.getArmLength() - target) /
           (ArmConstants.TELE_FORWARD_SOFT_LIMIT - ArmConstants.TELE_REVERSE_SOFT_LIMIT);
  }

  @Override
  public void runWithOutput(double output) {
    arm.setTeleVel(output);
  }
}
