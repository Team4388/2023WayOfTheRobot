// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveModule extends SubsystemBase {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private CANCoder canCoder;

    public static Gains swerveGains = SwerveDriveConstants.PIDConstants.SWERVE_GAINS;
  
  
    /** Creates a new SwerveModule. */
    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder canCoder, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.slot0.kP = swerveGains.kP;
        angleConfig.slot0.kI = swerveGains.kI;
        angleConfig.slot0.kD = swerveGains.kD;

        // use the CANcoder as the remote sensor for the primary TalonFX PID
        angleConfig.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        angleConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleConfig);

        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.magnetOffsetDegrees = offset;
        canCoder.configAllSettings(canCoderConfig);
    }

    /**
     * Get the angle of a SwerveModule as a Rotation2d
     * @return the angle of a SwerveModule as a Rotation2d
     */
    public Rotation2d getAngle() {
        // Note: This assumes that the CANCoders are setup with the default feedback coefficient and the sensor value reports degrees. 
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    /**
     * Set the speed and rotation of the SwerveModule from a SwerveModuleState object
     * @param desiredState a SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = this.getAngle();

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // calculate the difference between our current rotational position and our new rotational position
        Rotation2d rotationDelta = state.angle.minus(currentRotation);

        // calculate the new absolute position of the SwerveModule based on the difference in rotation
        double deltaTicks = (rotationDelta.getDegrees() / 360.) * SwerveDriveConstants.Conversions.CANCODER_TICKS_PER_ROTATION;

        // convert the CANCoder from its position reading to ticks
        double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
        angleMotor.set(TalonFXControlMode.Position, currentTicks + deltaTicks);

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
        driveMotor.set(angleMotor.get() + feetPerSecond / SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND);
    }

}
