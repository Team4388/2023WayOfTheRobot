package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc4388.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private WPI_TalonFX m_tele;
    private WPI_TalonFX m_pivot;
    private boolean m_debug;

    // Moves arm to distance [dist] then returns new ang
    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele, boolean debug) {
        m_tele = tele;
        m_pivot = pivot;
        
        tele.configFactoryDefault();
        pivot.configFactoryDefault();
    }

    public Arm(WPI_TalonFX pivot, WPI_TalonFX tele) {
        this(pivot, tele, false);
    }

    public void armSetRotation(double rot) {
        if (rot > 1 || rot < 0) return;
        // Move arm code
        m_pivot.set(ControlMode.Position, rot * (ArmConstants.PIVOT_REVERSE_SOFT_LIMIT - ArmConstants.PIVOT_FORWARD_SOFT_LIMIT) +
            ArmConstants.PIVOT_FORWARD_SOFT_LIMIT);
    }

    public void armSetLength(double len) {
        if (len > 1 || len < 0) return;
        // Move arm code
        m_tele.set(ControlMode.Position, len * (ArmConstants.TELE_REVERSE_SOFT_LIMIT - ArmConstants.TELE_FORWARD_SOFT_LIMIT) +
            ArmConstants.TELE_FORWARD_SOFT_LIMIT);
    }

    public double getArmLength() {
        return (m_tele.getSelectedSensorPosition() - ArmConstants.TELE_FORWARD_SOFT_LIMIT) /
            (ArmConstants.TELE_REVERSE_SOFT_LIMIT - ArmConstants.TELE_FORWARD_SOFT_LIMIT);
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
