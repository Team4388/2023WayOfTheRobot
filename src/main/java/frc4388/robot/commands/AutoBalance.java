// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.RobotGyro;

public class AutoBalance extends PelvicInflammatoryDisease {
	RobotGyro gyro;
	SwerveDrive drive;

	/** Creates a new AutoBalanceTF2. */
	public AutoBalance(RobotGyro gyro, SwerveDrive drive) {
		super(1.0, 0, 0, 0);

		this.gyro = gyro;
		this.drive = drive;

		addRequirements(drive);
	}

	@Override
	public double getError() {
		var pitch = gyro.getRoll();
		SmartDashboard.putNumber("pitch", pitch);
		return pitch;
	}

	@Override
	public void runWithOutput(double output) {
		double out2 = MathUtil.clamp(output / 40, -.5, .5);

		if (Math.abs(getError()) < 3) out2 = 0;
		drive.driveWithInput(new Translation2d(0, out2), new Translation2d(), false);
	}

	@Override
	public void initialize() {
		super.initialize();
		// this.gyro.reset();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
