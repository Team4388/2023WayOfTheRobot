// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
	private Solenoid m_clawSolenoid;

	/** Creates a new Claw. */
	public Claw(Solenoid clawSolenoid) {
		m_clawSolenoid = clawSolenoid;
	}

	void setClaw(boolean open) {
		m_clawSolenoid.set(open);
	}

	@Override
	public void periodic() {
	  
	}
}
