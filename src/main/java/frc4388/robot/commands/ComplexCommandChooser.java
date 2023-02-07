// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.Collections;
import java.util.HashMap;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ComplexCommandChooser extends CommandBase {

  private HashMap<Command, BooleanSupplier> commandMap;

  /** Creates a new CommandChooser.
   * @author Aarav Shah
   * @author Daniel Thomas McGrath
  */
  public ComplexCommandChooser(HashMap<Command, BooleanSupplier> commandMap) {
    this.commandMap = commandMap;

    Set<Subsystem> allReqs = Collections.emptySet();

    for(Command command : commandMap.keySet())
      allReqs.addAll(command.getRequirements());

    addRequirements(allReqs.toArray(Subsystem[]::new));
  }

  /**
   * Runs an operation on every command in the group
   * 
   * @param consumer operation to run
   */
  public void runCommands(Consumer<Command> consumer) {
    Set<Subsystem> reqCheck = Collections.emptySet();
    
    for(Command command : commandMap.keySet()) {
      boolean reqFree = true;
      for(Subsystem req : (Subsystem[]) command.getRequirements().toArray())
        reqFree &= !reqCheck.contains(req);

      if(commandMap.get(command).getAsBoolean() && reqFree) {
        consumer.accept(command);
        reqCheck.addAll(command.getRequirements());
      }
    }
  }

  @Override
  public void initialize() { runCommands(c -> c.initialize()); }

  @Override
  public void execute() { runCommands(c -> c.execute()); }

  @Override
  public void end(boolean interrupted) { runCommands(c -> c.end(interrupted)); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = true;

    // Checks that everything is finished
    //! command will not finish if there is an unfinished command
    for(Command command : commandMap.keySet())
      if(commandMap.get(command).getAsBoolean()) finished &= command.isFinished();
    
    return finished;
  }
}