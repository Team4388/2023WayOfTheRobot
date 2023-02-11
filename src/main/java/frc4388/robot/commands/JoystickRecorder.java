// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
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
  
  // HashMap<Long, double[]> timedInput;
  ArrayList<double[]> outputs;

  private final long startTime;


  /** Creates a new JoystickRecorder. */
  public JoystickRecorder(SwerveDrive swerve, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier, Supplier<Double> rightYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;

    this.startTime = System.currentTimeMillis();
    // this.timedInput = new HashMap<Long, double[]>();
    this.outputs = new ArrayList<double[]>();

    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timedInput.put((long) 0, new double[] {0.0, 0.0, 0.0, 0.0});
    outputs.add(new double[] {0.0, 0.0, 0.0, 0.0});

    System.out.println("STARTING RECORDING");
    System.out.println("STARTING RECORDING");
    System.out.println("STARTING RECORDING");
    System.out.println("STARTING RECORDING");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] inputs = new double[] {leftXSupplier.get(), leftYSupplier.get(), rightXSupplier.get(), rightYSupplier.get()};
    // timedInput.put(System.currentTimeMillis() - startTime, inputs);
    outputs.add(inputs);

    swerve.driveWithInput(new Translation2d(inputs[0], inputs[1]), new Translation2d(-inputs[2], inputs[3]), true);

    System.out.println("RECORDING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    File output = new File("/home/lvuser/JoystickInputs.txt");

    try(PrintWriter writer = new PrintWriter(output)) {
      for(double[] input : outputs) {
        writer.println(input[0] + "," + input[1] + "," + input[2] + "," + input[3]);
      }
      writer.close();
    } catch(IOException e) {
        e.printStackTrace();
    }

    System.out.println("STOPPED RECORDING");
    System.out.println("STOPPED RECORDING");
    System.out.println("STOPPED RECORDING");
    System.out.println("STOPPED RECORDING");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
