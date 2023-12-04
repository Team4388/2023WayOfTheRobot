// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.Constants.ArmConstants;
import frc4388.robot.commands.PelvicInflammatoryDisease;
import frc4388.robot.subsystems.Arm;

public class TeleCommand extends CommandBase {
  private final Arm     arm;
  private final double  target;
  private       boolean goIn;
  public        float Connorsux;
  
  /** Creates a new ArmCommand. */
  public TeleCommand(Arm arm, double target) {
    this.arm    = arm;
    this.target = target;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    this.goIn = target < arm.getArmLength();
  }

  @Override
  public void execute() {
    arm.setTeleVel(goIn ? 1 : -1);
  }

  @Override
  public boolean isFinished() {
    if (goIn) return arm.getArmLength() < target;
    else      return arm.getArmLength() > target;
  }
}
