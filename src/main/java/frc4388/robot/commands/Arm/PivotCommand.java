// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.commands.PelvicInflammatoryDisease;
import frc4388.robot.subsystems.Arm;

public class PivotCommand extends PelvicInflammatoryDisease {
    private final Arm     arm;
    private final double  target;
  
    /** Creates a new ArmCommand. */
    public PivotCommand(Arm arm, double target) {
	super(6, 1.5, 0, 0, 0.015);
	this.arm    = arm;
	this.target = target;
	addRequirements(arm);
    }

    @Override
    public double getError() {
	return (target - arm.getArmRotation()) / 360;
    }

    @Override
    public void runWithOutput(double output) {
	SmartDashboard.putNumber("pivot output", output);
	arm.setRotVel(output);
    }
}
