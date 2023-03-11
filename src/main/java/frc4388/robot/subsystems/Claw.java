package frc4388.robot.subsystems;
import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
	private PWM m_clawMotor;
	private boolean m_open     = false;
	private boolean m_disabled = false;

    // Opens claw
	public Claw(PWM m_clawMotor) {
		 this.m_clawMotor = m_clawMotor;
		 setClaw(true);
	}

    public void setClaw(boolean open) {
		if (m_disabled) return;
        // Open claw
		// m_clawMotor.setRaw(150);
		m_open = open;
		System.out.println("setClaw()");
		// m_clawMotor.setPosition(0.5);
		// m_clawMotor.setRaw(0);
		// m_clawMotor.setRaw(m_open ? 0 : 255);
		// m_clawMotor.setSpeed(m_open ? -1 : 1);
		PWMJNI.setPWMSpeed(m_clawMotor.getHandle(), m_open ? -1 : 1);
		// PWMJNI.setPWMDisabled(0);
		System.out.println("Claw Pos: " + m_clawMotor.getRaw());
    }

	public void toggle() {
		System.out.println("toggle()");
		setClaw(!m_open);
	}

	public boolean isClawOpen() {
		return m_open;
	}

	public void disable() {
		m_disabled = true;
		// PWMJNI.setPWMRaw(m_clawMotor.getHandle(), PWMJNI.getPWMRaw(m_clawMotor.getHandle()));
		PWMJNI.setPWMSpeed(m_clawMotor.getHandle(), 0.5);
		// PWMJNI.setPWMDisabled(m_clawMotor.getHandle());
	}

	public void enable() {
		m_disabled = false;
	}
}