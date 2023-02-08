// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.utility.controller.DeadbandedXboxController;

public class JoystickRecorder extends CommandBase {

  DeadbandedXboxController joystick;
  Supplier<DeadbandedXboxController> joystickSupplier;

  int numEntries = 0;

  /** Creates a new JoystickRecorder. */
  public JoystickRecorder(Supplier<DeadbandedXboxController> joystickSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystickSupplier = joystickSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    joystick = joystickSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX = joystick.getLeftX();
    double leftY = joystick.getLeftY();
    double rightX = joystick.getRightX();
    double rightY = joystick.getRightY();

    try {
      Files.writeString(Path.of("JoystickInputs.txt"), "leftX: " + leftX + ", " + "leftY: " + leftY + ", " + "rightX: " + rightX + ", " + "rightY: " + rightY);
      numEntries = numEntries + 1;
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numEntries > 10;
  }
}
