// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.Constants.VisionConstants;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;

public class LimeAlign extends PelvicInflammatoryDisease {

  SwerveDrive drive;
  Limelight lime;

  public LimeAlign(SwerveDrive drive, Limelight lime) {
    super(0.7, 0.1, 0.0, 0.0, 0);

    this.drive = drive;
    this.lime = lime;
    
    addRequirements(drive, lime);
  }

  @Override
  public double getError() {
    double err = 0.0;

    try {
      err = lime.getFirstTargetPoint().getYaw() / (VisionConstants.H_FOV / 2); 
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
