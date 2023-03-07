// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Arm;
import frc4388.robot.subsystems.Claw;

public class ArmCommand extends PelvicInflammatoryDisease {
  private Arm arm;
  private Claw claw;
  private boolean toggle;
  
  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm, Claw claw, boolean open) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(0.6, 0, 0, 0);
    this.arm = arm;
    this.claw = claw;
    this.toggle = open;
    addRequirements(arm, claw);
  }

  public ArmCommand(Arm arm, Claw claw) {
    this(arm, claw, claw.isClawOpen());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public double getError() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void runWithOutput(double output) {
    // TODO Auto-generated method stub
    
  }
}
