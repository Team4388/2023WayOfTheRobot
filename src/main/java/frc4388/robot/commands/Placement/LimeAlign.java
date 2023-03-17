// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Placement;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.commands.PelvicInflammatoryDisease;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;

public class LimeAlign extends PelvicInflammatoryDisease {

  SwerveDrive drive;
  Limelight lime;

  DoubleSupplier ds;

  public LimeAlign(SwerveDrive drive, Limelight lime, DoubleSupplier ds) {
    super(0.7, 0.4, 0.0, 0.0, 0.04);

    this.drive = drive;
    this.lime = lime;
    this.ds = ds;
    
    addRequirements(drive, lime);
  }

  @Override
  public double getError() {

    if (lime.getNumTapes() > 2) {
      return 0.0;
    }

    double err = 0.0;

    try {
      err = ds.getAsDouble() / (VisionConstants.H_FOV / 2); 
    } catch (NullPointerException ex) {}
    
    return err;
  }

  @Override
  public void runWithOutput(double output) {

    if (output > 0) {
      output += 0.6;
    } else if (output < 0) {
      output -= 0.6;
    }

    drive.driveWithInput(new Translation2d(output, 0.0), new Translation2d(0.0, 0.0), true);
  }
}
