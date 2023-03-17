// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Placement;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PelvicInflammatoryDisease;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;

public class DriveToLimeDistance extends PelvicInflammatoryDisease {

  SwerveDrive drive;
  Limelight lime;

  double targetDistance;
  DoubleSupplier ds;

  /** Creates a new DriveToLimeDistance. */
  public DriveToLimeDistance(SwerveDrive drive, Limelight lime, double targetDistance, DoubleSupplier ds) {
    super(0.5, 0.0, 0.0, 0.0, 1);

    this.drive = drive;
    this.lime = lime;
    this.targetDistance = targetDistance;
    this.ds = ds;

    addRequirements(drive, lime);
  }

  @Override
  public double getError() {
    return targetDistance - ds.getAsDouble();
  }

  @Override
  public void runWithOutput(double output) {
    System.out.println(output / Math.abs(getError()));
    drive.driveWithInput(new Translation2d(0.0, output / Math.abs(getError())), new Translation2d(0.0, 0.0), true);    
  }
}
