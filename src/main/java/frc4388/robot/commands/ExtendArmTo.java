// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Arm;

public class ExtendArmTo extends CommandBase {
    private final Arm     m_arm;
    private final double  m_target;
    private final boolean m_direction;

    public ExtendArmTo(Arm arm, double units) {
        m_arm       = arm;
        m_target    = units;
        m_direction = m_target > 1;
        addRequirements(m_arm);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_arm.setTeleVel(m_direction ? 1 : -1, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_direction)
            return m_arm.getTeleUnit() > m_target;
        else
            return m_arm.getTeleUnit() < m_target;
    }
}
