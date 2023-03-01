package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import frc4388.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private WPI_TalonFX m_tele;
    private WPI_TalonFX m_pivot;
    private CANCoder    m_pivotEncoder;
    private boolean     m_debug;

    // Moves arm to distance [dist] then returns new ang
    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele, CANCoder encoder, boolean debug) {
        m_tele         = tele;
        m_pivot        = pivot;
        m_pivotEncoder = encoder;

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.slot0.kP = ArmConstants.kP;
        pivotConfig.slot0.kI = ArmConstants.kI;
        pivotConfig.slot0.kD = ArmConstants.kD;

        pivotConfig.remoteFilter0.remoteSensorDeviceID = encoder.getDeviceID();
        pivotConfig.remoteFilter0.remoteSensorSource   = RemoteSensorSource.CANCoder;
        pivotConfig.primaryPID.selectedFeedbackSensor  = FeedbackDevice.RemoteSensor0;
        pivot.configAllSettings(pivotConfig);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.magnetOffsetDegrees = ArmConstants.OFFSET;
        m_pivotEncoder.configAllSettings(config);
        
        tele.configFactoryDefault();
        pivot.configFactoryDefault();
    }

    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele, CANCoder encoder) {
        this(pivot, tele, encoder, false);
    }

    public void setRotVel(double vel) {
        m_pivot.set(ControlMode.Velocity, vel);
    }

    public void setTeleVel(double vel) {
        m_tele.set(ControlMode.Velocity, vel);
    }

    public void armSetRotation(double rot) {
        if (rot > 1 || rot < 0) return;
        // Move arm code
        m_pivot.set(ControlMode.Position, rot * Math.abs(ArmConstants.PIVOT_REVERSE_SOFT_LIMIT - ArmConstants.PIVOT_FORWARD_SOFT_LIMIT) +
            ArmConstants.PIVOT_FORWARD_SOFT_LIMIT);
    }

    public void armSetLength(double len) {
        if (len > 1 || len < 0) return;
        // Move arm code
        m_tele.set(ControlMode.Position, len * Math.abs(ArmConstants.TELE_REVERSE_SOFT_LIMIT - ArmConstants.TELE_FORWARD_SOFT_LIMIT) +
            ArmConstants.TELE_FORWARD_SOFT_LIMIT);

        if (m_tele.isRevLimitSwitchClosed() == 1) {
            m_tele.setSelectedSensorPosition(ArmConstants.TELE_REVERSE_SOFT_LIMIT);
        } else if (m_tele.isFwdLimitSwitchClosed() == 1) {
            m_tele.setSelectedSensorPosition(ArmConstants.TELE_FORWARD_SOFT_LIMIT);
        }
    }

    public double getArmLength() {
        return (m_tele.getSelectedSensorPosition() - ArmConstants.TELE_FORWARD_SOFT_LIMIT) /
            (ArmConstants.TELE_REVERSE_SOFT_LIMIT - ArmConstants.TELE_FORWARD_SOFT_LIMIT);
    }

    public double getArmRotation() {
        return (m_pivotEncoder.getAbsolutePosition() - ArmConstants.PIVOT_FORWARD_SOFT_LIMIT) /
            (ArmConstants.PIVOT_REVERSE_SOFT_LIMIT - ArmConstants.PIVOT_FORWARD_SOFT_LIMIT);
    }

    public void runPivotTele(double pivot, double tele) {
        var rot = 0;

        if (checkLimits(tele, rot)) {
            armSetRotation(pivot);
            armSetLength(tele);
        }
    }

    /**
     * Checks that an input is within bounds
     * @param _len length from 0 to 1
     * @param _theta radians from the zero (straight up)
     * @return
     */
    public static boolean checkLimits(double _len, double _theta) {
        var len = _len * (ArmConstants.MAX_ARM_LEN - ArmConstants.MIN_ARM_LEN) + ArmConstants.MIN_ARM_LEN;
        var x   = len * Math.cos(_theta);
        var y   = ArmConstants.ARM_HEIGHT + len * Math.sin(_theta);

        var minHeight = Math.pow(ArmConstants.CURVE_POWER, Math.abs(x));
        if (y < minHeight)
            return false;
        
        return true;
    }

    @Override
    public void periodic() {
        if (m_debug)
            SmartDashboard.putNumber("Arm Motor", m_tele.getSelectedSensorPosition());
    }
}
