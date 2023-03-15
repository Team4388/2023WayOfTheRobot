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
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Pair;
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

  public ArrayList<Point> getTargetPoints() throws AbhiIsADumbass {
    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) throw new AbhiIsADumbass();

    ArrayList<Point> points = new ArrayList<>(2);

    for(PhotonTrackedTarget target : result.getTargets()){
      List<TargetCorner> corners = target.getDetectedCorners();
  
      double sumX = 0.0;
      double sumY = 0.0;
      double mx = 0.0;
      double my = 0.0;
  
      for (TargetCorner c : corners) {
        sumX += c.x;
        sumY += c.y;
      }
  
      mx = sumX / 4.0;
      my = sumY / 4.0;

      points.add(new Point(mx, my));
    }

    return points;
  }

  private double getPointAngle(Point point) {
    return (VisionConstants.LIME_VIXELS - point.y) * (VisionConstants.V_FOV / VisionConstants.LIME_VIXELS);
  }

  public double getHorizontalDistanceToTarget(boolean high) throws AbhiIsADumbass {
    ArrayList<Point> targetPoints = getTargetPoints();

    Point highPoint = targetPoints.get(0).y <= targetPoints.get(1).y ? targetPoints.get(0) : targetPoints.get(1);
    Point midPoint = targetPoints.get(0).y >= targetPoints.get(1).y ? targetPoints.get(0) : targetPoints.get(1);

    Point tapePoint = high ? highPoint : midPoint;
    double tapeHeight = high ? VisionConstants.HIGH_TAPE_HEIGHT : VisionConstants.MID_TAPE_HEIGHT;

    double theta = VisionConstants.LIME_ANGLE + getPointAngle(tapePoint);

    double effectiveTapeHeight = tapeHeight - VisionConstants.LIME_HEIGHT;

    double distanceToTape = effectiveTapeHeight / Math.sin(Math.toRadians(theta));

    double horizontalDistanceToTarget = Math.sqrt(Math.pow(distanceToTape, 2) - Math.pow(effectiveTapeHeight, 2));

    return horizontalDistanceToTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
