package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc4388.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private WPI_TalonFX m_armmotor;
    private boolean m_debug;
    // Moves arm to distence [dist] then returns new ang
    public Arm(WPI_TalonFX armmotor, boolean debug) {
        armmotor.configFactoryDefault();
        m_armmotor = armmotor;
    }

    public Arm(WPI_TalonFX armmoter) {
        this(armmoter, false);
    }

    public void armSetLength(double len) {
        if (len > 1 || len < 0) return;
        // Move arm code
        m_armmotor.set(ControlMode.Position, len * (ArmConstants.MAX_ARM_LEN - ArmConstants.MIN_ARM_LEN) +
            ArmConstants.MIN_ARM_LEN);
    }

    public double getArmLength() {
        return (m_armmotor.getSelectedSensorPosition() - ArmConstants.MIN_ARM_LEN) /
            (ArmConstants.MAX_ARM_LEN - ArmConstants.MIN_ARM_LEN);
    }

    @Override
    public void periodic() {
        if (m_debug)
            SmartDashboard.putNumber("Arm Motor", m_armmotor.getSelectedSensorPosition());
    }
}
