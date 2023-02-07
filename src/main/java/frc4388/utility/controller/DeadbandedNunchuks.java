// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility.controller;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.Constants.OIConstants;

/** Add your docs here. */
public class DeadbandedNunchuks extends Nunchuks {
    public DeadbandedNunchuks(int port) { super(port); }
  
    @Override public double getLeftX() { return getLeft().getX(); }
    @Override public double getLeftY() { return getLeft().getY(); }
    @Override public double getRightX() { return getRight().getX(); }
    @Override public double getRightY() { return getRight().getY(); }
  
    public Translation2d getLeft() { return skewToDeadzonedCircle(super.getLeftX(), super.getLeftY()); }
    public Translation2d getRight() { return skewToDeadzonedCircle(-super.getRightX(), super.getRightY()); }
  
    public static Translation2d skewToDeadzonedCircle(double x, double y) {
      Translation2d translation2d = new Translation2d(x, y);
      double magnitude = translation2d.getNorm();
      if (OIConstants.SKEW_STICKS && magnitude >= 1) return translation2d.div(magnitude);
      if (magnitude < OIConstants.LEFT_AXIS_DEADBAND) return new Translation2d(0,0);
      return translation2d;
    }
}
