// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {
  private PhotonCamera cam;

  private boolean lightOn;

  /** Creates a new Limelight. */
  public Limelight() {
    cam = new PhotonCamera(VisionConstants.NAME);
    cam.setDriverMode(false);
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

  // ! might need to find midpoint instead of entire target
  public PhotonTrackedTarget getAprilPoint() {
    setToAprilPipeline();

    if (!cam.isConnected()) return null;

    PhotonPipelineResult result = cam.getLatestResult();

    if (!result.hasTargets()) return null;

    return result.getBestTarget();
  }

  public double getDistanceToApril() {
    PhotonTrackedTarget aprilPoint = getAprilPoint();
    if (aprilPoint == null) return -1;

    double aprilHeight = VisionConstants.APRIL_HEIGHT - VisionConstants.LIME_HEIGHT;
    double theta = 35.0 + aprilPoint.getPitch();

    double distanceToApril = aprilHeight / Math.tan(Math.toRadians(theta));
    return distanceToApril;
  }

  public PhotonTrackedTarget getLowestTape() {
    setToLimePipeline();

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

  public int getNumTapes() {
    setToLimePipeline();

    PhotonPipelineResult result = cam.getLatestResult();

    return result.getTargets().size();
  }

  public double getDistanceToTape() {
    PhotonTrackedTarget tapePoint = getLowestTape();
    if (tapePoint == null) return -1;

    double tapeHeight = VisionConstants.MID_TAPE_HEIGHT - VisionConstants.LIME_HEIGHT;
    double theta = 35.0 + tapePoint.getPitch();

    double distanceToTape = tapeHeight / Math.tan(Math.toRadians(theta));
    return distanceToTape;
  }

  int ctr = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (ctr % 50 == 0) {
      SmartDashboard.putNumber("Horizontal Distance", getDistanceToTape());
    }

    ctr++;

  }
}
