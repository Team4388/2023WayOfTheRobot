// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.RobotTime;
import frc4388.utility.controller.DeadbandedXboxController;

public class JoystickRecorder extends CommandBase {
  
  SwerveDrive swerve;

  Supplier<Double> leftXSupplier;
  Supplier<Double> leftYSupplier;
  Supplier<Double> rightXSupplier;
  Supplier<Double> rightYSupplier;
  
  HashMap<Long, double[]> timedInput;

  private long startTime;


  /** Creates a new JoystickRecorder. */
  public JoystickRecorder(SwerveDrive swerve, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier, Supplier<Double> rightYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timedInput.put((long) 0, new double[] {0.0, 0.0, 0.0, 0.0});
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] inputs = new double[] {leftXSupplier.get(), leftYSupplier.get(), rightXSupplier.get(), rightYSupplier.get()};
    timedInput.put(System.currentTimeMillis() - startTime, inputs);

    swerve.driveWithInput(new Translation2d(inputs[0], inputs[1]), new Translation2d(-inputs[2], inputs[3]), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    File output = new File("JoystickInputs.txt");

    try(PrintWriter writer = new PrintWriter(output)) {
      for(long millis : timedInput.keySet()) {
        writer.println("time: " + millis + ", leftX: " + timedInput.get(millis)[0] + ", leftY: " + timedInput.get(millis)[1] + ", rightX: " + timedInput.get(millis)[2] + ", rightY: " + timedInput.get(millis)[3]);
      }
      writer.close();
    } catch(IOException e) {
        e.printStackTrace();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
