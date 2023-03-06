package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import frc4388.robot.Constants.ArmConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
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

        tele.configFactoryDefault();
        m_pivot.configFactoryDefault();

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.slot0.kP = ArmConstants.kP;
        pivotConfig.slot0.kI = ArmConstants.kI;
        pivotConfig.slot0.kD = ArmConstants.kD;

        pivotConfig.remoteFilter0.remoteSensorDeviceID = encoder.getDeviceID();
        pivotConfig.remoteFilter0.remoteSensorSource   = RemoteSensorSource.CANCoder;
        pivotConfig.primaryPID.selectedFeedbackSensor  = FeedbackDevice.RemoteSensor0;
        m_pivot.configAllSettings(pivotConfig);

        resetTeleSoftLimit();

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.magnetOffsetDegrees = ArmConstants.OFFSET;
        m_pivotEncoder.configAllSettings(config);
    }

    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele, CANCoder encoder) {
        this(pivot, tele, encoder, false);
    }

    public void setRotVel(double vel) {
        m_pivot.set(ControlMode.PercentOutput, vel / 5);
    }

    public void setTeleVel(double vel) {
        m_tele.set(ControlMode.PercentOutput, -0.25 * vel);
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
        double rot = 0;

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

    boolean tele_softLimit = false;
    public void resetTeleSoftLimit() {
        if (!tele_softLimit) {
            var tele_soft = m_tele.getSelectedSensorPosition();
            m_tele.configForwardSoftLimitThreshold(91000 - tele_soft);
            m_tele.configReverseSoftLimitThreshold(tele_soft);
            m_tele.configForwardSoftLimitEnable(true);
            m_tele.configReverseSoftLimitEnable(true);
        } else {
            m_tele.configForwardSoftLimitEnable(false);
            m_tele.configReverseSoftLimitEnable(false);
        }

        tele_softLimit = !tele_softLimit;
    }

    boolean resetable = true;

    @Override
    public void periodic() {
        double degrees = Math.abs(m_pivotEncoder.getAbsolutePosition() - 135);
        if (degrees < 2 && resetable) {
            var pivot_soft = m_pivot.getSelectedSensorPosition();
            var tele_soft  = m_tele.getSelectedSensorPosition();
            
            SmartDashboard.putNumber("start pivot", pivot_soft);
            SmartDashboard.putNumber("start tele", tele_soft);
            
            m_pivot.configForwardSoftLimitEnable(true);
            m_pivot.configReverseSoftLimitEnable(true);
            SmartDashboard.putNumber("fwd err", m_pivot.configForwardSoftLimitThreshold(1200 + pivot_soft).value);
            SmartDashboard.putNumber("rvs err", m_pivot.configReverseSoftLimitThreshold(pivot_soft).value);
            resetable = false;
        } else if (degrees > 2) {
            resetable = true;
        }

        double x = Math.cos(degrees);
        SwerveDriveConstants.ROTATION_SPEED = SwerveDriveConstants.MIN_ROT_SPEED + x * (SwerveDriveConstants.MAX_ROT_SPEED - SwerveDriveConstants.MIN_ROT_SPEED);
        // if (m_debug)
        //     SmartDashboard.putNumber("Arm Motor", m_tele.getSelectedSensorPosition());
        SmartDashboard.putNumber("Pivot CANCoder", m_pivotEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Pivot IntegratedSensor", m_pivot.getSelectedSensorPosition());
        SmartDashboard.putNumber("Telescope Encoder", m_tele.getSelectedSensorPosition());
    }
}
