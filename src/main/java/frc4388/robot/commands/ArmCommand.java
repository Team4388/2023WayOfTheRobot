// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import org.opencv.osgi.OpenCVInterface;

import frc4388.robot.subsystems.Arm;
import frc4388.robot.subsystems.Claw;

public class ArmCommand extends PelvicInflammatoryDisease {
  private final Arm     arm;
  private final Claw    claw;
  private final boolean toggle;
  private final double  target;
  
  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm, Claw claw, double target, boolean open) {
    super(0.6, 0, 0, 0, 0);
    this.arm    = arm;
    this.claw   = claw;
    this.toggle = open;
    this.target = target;
    addRequirements(arm, claw);
  }

  public ArmCommand(Arm arm, Claw claw, double target) {
    this(arm, claw, target, claw.isClawOpen());
  }

  @Override
  public double getError() {
    return (arm.getArmRotation() - target) / 360;
  }

  @Override
  public void runWithOutput(double output) {
    arm.setRotVel(output);
  }

  @Override
  public void end(boolean interrupted) {
    claw.setClaw(toggle);
  }
}
