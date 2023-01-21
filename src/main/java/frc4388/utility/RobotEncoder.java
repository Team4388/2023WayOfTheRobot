// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import com.ctre.phoenix.sensors.CANCoder;

/** Add your docs here. */
public class RobotEncoder extends CANCoder {
    private double offset;

    public RobotEncoder(int id, double offset) {
        super(id);

        this.offset = offset;
    }

    @Override
    public double getAbsolutePosition() {
        return super.getAbsolutePosition() - this.offset;
    }
}
