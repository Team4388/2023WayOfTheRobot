package frc4388.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

	private final PWM         m_leftMotor;
	private final PWM         m_rightMotor;
	private final CANSparkMax m_spinnyspin;

	private boolean m_open = false;

    // Opens claw
	public Claw(PWM leftMotor, PWM rightMotor, CANSparkMax spinnyspin) {
		m_leftMotor  = leftMotor;
		m_rightMotor = rightMotor;
		m_spinnyspin = spinnyspin;
		
		setClaw(true);
	}

    public void setClaw(boolean open) {
        // Open claw
		m_open = open;
		m_leftMotor.setRaw(m_open ? 1000 : 2000);
		m_rightMotor.setRaw(m_open ? 2000 : 1000);

		if (!m_open) {
			m_spinnyspin.set(-0.2);
			
			new Timer().schedule(new TimerTask() {
				@Override
				public void run() {
					nospinnyspin();
				}
			}, 750);
		}
    }

	public void toggle() {
		setClaw(!m_open);
	}

	public boolean isClawOpen() {
		return m_open;
	}

	public void yesspinnyspin() {
		m_spinnyspin.set(0.2);
	}

	public void nospinnyspin() {
		m_spinnyspin.set(0);
	}
}
