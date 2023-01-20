// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.math.MathUtil;
import frc4388.robot.Robot;

// NOTE:	Consider using this command inline, rather than writing a subclass.	For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceTF2 extends PelvicInflamitoryDisease {
	Robot.MicroBot bot;

	/** Creates a new AutoBalanceTF2. */
	public AutoBalanceTF2(Robot.MicroBot bot) {
		super(.7, .1, 15, 0);
		addRequirements(bot);
		this.bot = bot;
	}

	@Override
	public double getError() {
		return bot.gyro.getPitch();
	}

	@Override
	public void runWithOutput(double output) {
		double out2 = MathUtil.clamp(output / 40, -.5, .5);
		if (Math.abs(bot.gyro.getPitch()) < 3) out2 = 0;
		bot.setOutput(out2);
	}

	@Override
	public void initialize() {
		super.initialize();
		this.bot.gyro.reset();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
