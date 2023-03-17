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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VisionConstants;
import frc4388.utility.AbhiIsADumbass;

public class Limelight extends SubsystemBase {
  private PhotonCamera cam;

  private boolean lightOn;

  /** Creates a new Limelight. */
  public Limelight() {
    cam = new PhotonCamera(VisionConstants.NAME);
    cam.setDriverMode(false);
    
    setToLimePipeline();
  }

  public void setLEDs(boolean on) {
    lightOn = on;
    cam.setLED(lightOn ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void toggleLEDs() {
    lightOn = !lightOn;
    cam.setLED(lightOn ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setDriverMode(boolean driverMode) {
    cam.setDriverMode(driverMode);
  }

  public void setToLimePipeline() {
    cam.setPipelineIndex(1);
  }

  public void setToAprilPipeline() {
    cam.setPipelineIndex(0);
  }

  public ArrayList<Point> getTargetPoints() {
    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    ArrayList<Point> points = new ArrayList<>(2);

    for(PhotonTrackedTarget target : result.getTargets()) {
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

  public PhotonTrackedTarget getFirstTargetPoint() {
    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    return result.getBestTarget();
  }

  public PhotonTrackedTarget getLowestTargetPoint() {
    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    ArrayList<PhotonTrackedTarget> points = (ArrayList<PhotonTrackedTarget>) result.getTargets();

    PhotonTrackedTarget lowest = points.get(0);
    for (PhotonTrackedTarget point : points) {
      if (point.getPitch() < lowest.getPitch()) {
        lowest = point;
      }
    }

    return lowest;
  }

  public int numTargets() {
    PhotonPipelineResult result = cam.getLatestResult();

    return result.getTargets().size();
  }

  public double getHorizontalDistanceToTarget(boolean high) {
    ArrayList<Point> targetPoints = getTargetPoints();
    if (targetPoints == null) return -1;

    // Point highPoint = targetPoints.get(0).y <= targetPoints.get(1).y ? targetPoints.get(0) : targetPoints.get(1);
    // Point midPoint = targetPoints.get(0).y >= targetPoints.get(1).y ? targetPoints.get(0) : targetPoints.get(1);

    PhotonTrackedTarget tapePoint = getFirstTargetPoint();//high ? highPoint : midPoint;
    double tapeHeight = VisionConstants.MID_TAPE_HEIGHT;//high ? VisionConstants.HIGH_TAPE_HEIGHT : VisionConstants.MID_TAPE_HEIGHT;

    double theta = 35.0 + tapePoint.getPitch();

    double effectiveTapeHeight = tapeHeight - VisionConstants.LIME_HEIGHT;

    double horizontalDistanceToTarget = effectiveTapeHeight / Math.tan(Math.toRadians(theta));

    return horizontalDistanceToTarget;
  }

  int ctr = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (ctr % 50 == 0) {
      SmartDashboard.putNumber("Horizontal Distance", getHorizontalDistanceToTarget(false));
    }

    ctr++;

  }
}
