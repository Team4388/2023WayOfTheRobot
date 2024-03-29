package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import frc4388.robot.Constants.ArmConstants;
import frc4388.utility.DeferredBlock;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private WPI_TalonFX m_tele;
    public WPI_TalonFX  m_pivot;
    private CANCoder    m_pivotEncoder;

    // Moves arm to distance [dist] then returns new ang
    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele, CANCoder encoder, boolean debug) {
        m_tele         = tele;
        m_pivot        = pivot;
        m_pivotEncoder = encoder;

        m_tele.configFactoryDefault();
        m_tele.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        m_tele.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        m_pivot.configFactoryDefault();

        // * Example of deferred code
        new DeferredBlock(() -> resetTeleSoftLimit());

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.magnetOffsetDegrees = ArmConstants.OFFSET;
        m_pivotEncoder.configAllSettings(config);
    }

    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele, CANCoder encoder) {
        this(pivot, tele, encoder, false);
    }

    public void setRotVel(double vel) {
        if (vel > 1) vel = 1;

        var degrees = Math.abs(getArmRotation()) - 135;
        SmartDashboard.putNumber("arm degrees", degrees);
        SmartDashboard.putNumber("arm rot vel", vel);

        if ((degrees < 2 && vel < 0) || (degrees > 110 && vel > 0)) {
            m_pivot.set(ControlMode.PercentOutput, 0);
        } else if (degrees > 90 && vel > 0) {
            m_pivot.set(ControlMode.PercentOutput, .15 * vel);
        } else if (degrees < 25 && vel < 0) {
            m_pivot.set(ControlMode.PercentOutput, .15 * vel);
        } else {
            m_pivot.set(ControlMode.PercentOutput, .3 * vel);
        }
    }

    public void setTeleVel(double vel) {
        m_tele.set(ControlMode.PercentOutput, -0.5 * vel);
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
        return m_tele.getSelectedSensorPosition() - tele_soft;
    }

    public double getArmRotation() {
        return m_pivotEncoder.getAbsolutePosition();
    }

    public void runPivotTele(double pivot, double tele) {
        double abs_pivot = Math.toRadians(getArmRotation() - 135);
        double abs_tele  = (getArmLength() - ArmConstants.TELE_REVERSE_SOFT_LIMIT) /
                           (ArmConstants.TELE_FORWARD_SOFT_LIMIT - ArmConstants.TELE_REVERSE_SOFT_LIMIT);

        if (pivot > 0 || tele < 0 || checkLimits(abs_tele, abs_pivot)) {
            setRotVel(pivot);
            setTeleVel(tele);
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
        return y < minHeight;
    }

    boolean tele_softLimit = false;
    double  tele_soft = 0;
    public void resetTeleSoftLimit() {
        if (!tele_softLimit) {
            tele_soft = m_tele.getSelectedSensorPosition();
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

    boolean resetable  = true;
    boolean tele_reset = true;

    @Override
    public void periodic() {

        if (m_tele.isFwdLimitSwitchClosed() == 1 && tele_reset) {
            var tele_soft = m_tele.getSelectedSensorPosition();
            m_tele.configForwardSoftLimitThreshold(91000 - tele_soft);
            m_tele.configReverseSoftLimitThreshold(1000  - tele_soft);
            m_tele.configForwardSoftLimitEnable(true);
            m_tele.configReverseSoftLimitEnable(true);
            tele_reset = false;
        } else if (m_tele.isFwdLimitSwitchClosed() == 0) {
            tele_reset = true;
        }

        // double x = Math.cos(Math.toRadians(degrees));
        SmartDashboard.putNumber("arm length", getArmLength());
    }

    public void killSoftLimits() {
        resetTeleSoftLimit();
    }
}