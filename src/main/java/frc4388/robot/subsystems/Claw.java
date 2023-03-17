package frc4388.robot.subsystems;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
	private PWM m_clawMotor;
	private boolean m_open     = false;

    // Opens claw
	public Claw(PWM m_clawMotor) {
		 this.m_clawMotor = m_clawMotor;
		 setClaw(true);
	}

    public void setClaw(boolean open) {
        // Open claw
		m_open = open;
		m_clawMotor.setRaw(m_open ? 1000 : 2000);
    }

	public void toggle() {
		setClaw(!m_open);
	}

	public boolean isClawOpen() {
		return m_open;
	}

}
