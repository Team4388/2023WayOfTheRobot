// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;

public class SwerveModule extends SubsystemBase {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private CANCoder canCoder;
  
  
    /** Creates a new SwerveModule. */
    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder canCoder, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        TalonFXConfiguration angleConfig = new TalonFXConfiguration();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
