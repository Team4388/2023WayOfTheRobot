// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4388.utility.RobotTime;

public class TimedCommand extends CommandBase {

  Supplier<Command> cs;
  double duration;

  long startTime;
  long deltaTime;

  /** Creates a new TimedCommand. Duration is in seconds. */
  public TimedCommand(Supplier<Command> cs, double duration) {
    this.cs = cs;
    this.duration = duration;


    Object[] reqs_obj = cs.get().getRequirements().toArray();
    Subsystem[] reqs = new Subsystem[reqs_obj.length];
    for (int i = 0; i < reqs.length; i++)
      reqs[i] = (Subsystem) reqs_obj[i];

    addRequirements(reqs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cs.get().initialize();

    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cs.get().execute();

    deltaTime = System.currentTimeMillis() - startTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (deltaTime / 1000.0) >= duration;
  }
}
