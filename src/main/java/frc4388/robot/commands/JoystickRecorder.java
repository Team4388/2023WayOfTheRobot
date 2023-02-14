// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;

public class JoystickRecorder extends CommandBase {
  
  SwerveDrive swerve;

  Supplier<Double> leftXSupplier;
  Supplier<Double> leftYSupplier;
  Supplier<Double> rightXSupplier;
  Supplier<Double> rightYSupplier;
  
  ArrayList<Object[]> outputs;

  private long startTime;


  /** Creates a new JoystickRecorder. */
  public JoystickRecorder(SwerveDrive swerve, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier, Supplier<Double> rightYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;

    this.outputs = new ArrayList<Object[]>();

    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();

    // timedInput.put((long) 0, new double[] {0.0, 0.0, 0.0, 0.0});
    outputs.add(new Object[] {(double) 0.0, (double) 0.0, (double) 0.0, (double) 0.0, (long) 0});

    System.out.println("STARTING RECORDING");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Object[] inputs = new Object[] {(double) leftXSupplier.get(), (double) leftYSupplier.get(), (double) rightXSupplier.get(), (double) rightYSupplier.get(), (long) (System.currentTimeMillis() - startTime)};
    outputs.add(inputs);

    swerve.driveWithInput(new Translation2d((double) inputs[0], (double) inputs[1]), new Translation2d((double) inputs[2], (double) inputs[3]), true);

    System.out.println("RECORDING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    File output = new File("/home/lvuser/JoystickInputs.txt");

    try(PrintWriter writer = new PrintWriter(output)) {
      
      for(Object[] input : outputs) {
        writer.println(input[0] + "," + input[1] + "," + input[2] + "," + input[3] + "," + input[4]);
      }

      writer.close();
    } catch(IOException e) {
        e.printStackTrace();
    }

    System.out.println("STOPPED RECORDING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
