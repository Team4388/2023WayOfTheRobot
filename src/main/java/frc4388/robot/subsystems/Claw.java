package frc4388.robot.subsystems;
import edu.wpi.first.wpilibj.PWM;

public class Claw {
	private PWM m_clawMotor;
	private boolean m_open = false;

    // Opens claw
	public Claw(PWM m_clawMotor) {
		 this.m_clawMotor = m_clawMotor;
		 setClaw(true);
	}

    public void setClaw(boolean open) {
        // Open claw
		m_open = open;
		m_clawMotor.setRaw(open ? 0 : 2000);
    }

	public boolean isClawOpen() {
		return m_open;
	}
}