// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveDrive;

public class DriveWithInput extends CommandBase {
  /** Creates a new DriveWithInput. */
  private final SwerveDrive swerve;

  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  private final Supplier<Double> rot;
  private final boolean fieldRelative;

  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;



  
  public DriveWithInput(SwerveDrive swerve, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot, boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.fieldRelative = fieldRelative;

    this.xLimiter = new SlewRateLimiter(3);
    this.yLimiter = new SlewRateLimiter(3);
    this.rotLimiter = new SlewRateLimiter(3);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();
    double r = rot.get();

    x = -xLimiter.calculate(MathUtil.applyDeadband(x * -0.3, 0.02) * Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    y = -yLimiter.calculate(MathUtil.applyDeadband(y * 0.3, 0.02) * Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    r = -rotLimiter.calculate(MathUtil.applyDeadband(r * 0.3, 0.02) * Units.feetToMeters(Math.PI));
  
    swerve.drive(x, y, r, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("----------------------------------------------------------------");
    System.out.println("DriveWithInput ended");
    System.out.println("Interrupted: " + interrupted);
    System.out.println("----------------------------------------------------------------");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
