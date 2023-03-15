// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VisionConstants;
import frc4388.utility.AbhiIsADumbass;

public class Limelight extends SubsystemBase {
  private PhotonCamera cam;
  /** Creates a new LImelight. */
  public Limelight() {
    cam = new PhotonCamera(VisionConstants.NAME);
  }

  public void setLEDs(boolean on) {
    cam.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setDriverMode(boolean driverMode) {
    cam.setDriverMode(driverMode);
  }

  public Point getTargetPoints() throws AbhiIsADumbass {
    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) throw new AbhiIsADumbass();

    Point point = new Point();
    List<TargetCorner> corners = result.getTargets().get(0).getDetectedCorners();

    double x1, x2, y1, y2;
    double mx, my;

    return new Point(mx, my);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
