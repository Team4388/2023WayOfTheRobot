// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Placement;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PelvicInflammatoryDisease;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;

public class AprilRotAlign extends PelvicInflammatoryDisease {
  
  SwerveDrive drive;
  Limelight lime;
  
  /** Creates a new AprilRotAlign. */
  public AprilRotAlign(SwerveDrive drive, Limelight lime) {
    super(0.1, 0.2, 0.0, 0.0, 0.0);
  
    this.drive = drive;
    this.lime = lime;

    addRequirements(drive, lime);
  }

  @Override
  public double getError() {
    double err = 0.0;

    try {
      err = lime.getAprilSkew();
    } catch (NullPointerException ex) {}
    
    return err;
  }

  @Override
  public void runWithOutput(double output) {
    drive.driveWithInput(new Translation2d(0.0, 0.0), new Translation2d(-output, 0.0), true);
  }
}
