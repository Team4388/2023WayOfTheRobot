// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.AbhiIsADumbass;

public class LimeAlign extends PelvicInflammatoryDisease {

  SwerveDrive drive;
  Limelight lime;


  public LimeAlign(SwerveDrive drive, Limelight lime) {
    super(0.1, 0.0, 0.0, 0.0, 10);

    this.drive = drive;
    this.lime = lime;
    
    addRequirements(drive, lime);
  }

  @Override
  public double getError() {
    try {
      return lime.getTargetPoints().get(0).x - (VisionConstants.LIME_HIXELS / 2);
    } catch (AbhiIsADumbass abhiIsADumbass) {
      abhiIsADumbass.printStackTrace();
      return 0;
    }
  }

  @Override
  public void runWithOutput(double output) {
    drive.driveWithInput(new Translation2d(output, 0.0), new Translation2d(0.0, 0.0), true);
  }
}
