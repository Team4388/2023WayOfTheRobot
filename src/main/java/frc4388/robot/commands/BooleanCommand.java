// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BooleanCommand extends CommandBase {

  private Supplier<Command> onTrue;
  private Supplier<Command> onFalse;

  private Supplier<Boolean> condition;

  private Supplier<Command> selected;


  /** Creates a new BooleanCommand. */
  public BooleanCommand(Supplier<Command> onTrue, Supplier<Command> onFalse, Supplier<Boolean> condition) {
    this.onTrue = onTrue;
    this.onFalse = onFalse;
    this.condition = condition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (condition.get()) {
      selected = onTrue;
    } else {
      selected = onFalse;
    }
    if (selected.get() != null) {
      selected.get().initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (selected.get() != null) {
      selected.get().execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (selected.get() != null) {
      selected.get().end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (selected.get() != null) {
      return selected.get().isFinished();
    } else {
      return true;
    }
  }
}
