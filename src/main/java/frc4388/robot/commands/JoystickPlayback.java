// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Scanner;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.controller.DeadbandedXboxController;

public class JoystickPlayback extends CommandBase {

  SwerveDrive swerve;
  Scanner input;

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      input = new Scanner(new File("JoystickInput.txt"));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    String line = "";
    if (input.hasNextLine()) {
      line = input.nextLine();
    }

    int ileftX = line.indexOf("leftX: ");
    int ileftY = line.indexOf(", leftY: ");
    int irightX = line.indexOf(", rightX: ");
    int irightY = line.indexOf(", rightY: ");

    double leftX = Double.parseDouble(line.substring(ileftX + 1, ileftY));
    double leftY = Double.parseDouble(line.substring(ileftY + 1, irightX));
    double rightX = Double.parseDouble(line.substring(irightX + 1, irightY));
    double rightY = Double.parseDouble(line.substring(irightY + 1));

    this.swerve.driveWithInput(new Translation2d(leftX, leftY), new Translation2d(-rightX, rightY), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    input.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
