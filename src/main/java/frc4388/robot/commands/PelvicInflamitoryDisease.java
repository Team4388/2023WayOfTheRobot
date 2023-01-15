// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.utility.Gains;

public abstract class PelvicInflamitoryDisease extends CommandBase {
	protected Gains gains;
	private double output = 0;

	/** Creates a new PelvicInflamitoryDisease. */
	public PelvicInflamitoryDisease(double kp, double ki, double kd, double kf) {
		gains = new Gains(kp, ki, kd, kf, 0);
	}

	public PelvicInflamitoryDisease(Gains gains) {
		this.gains = gains;
	}

	/** produces the error from the setpoint */
	public abstract double getError();
	/** figure it out bitch */
	public abstract void runWithOutput(double output);

	// Called when the command is initially scheduled.
	@Override 
	public void initialize() {
		output = 0;
	}

	private double prevError, cumError = 0;
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override 
	public void execute() {
		double error = getError();
		cumError += error * .02; // 20 ms
		double delta = error - prevError;

		output = error * gains.kP;
		output += cumError * gains.kI;
		output += delta * gains.kD;
		output += gains.kF;

		runWithOutput(output);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() { return false; }
}
