package frc4388.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

	// private final PWM         m_leftMotor;
	// private final PWM         m_rightMotor;
	private final Servo         m_leftMotor;
	private final Servo         m_rightMotor;
	private final CANSparkMax m_spinnyspin;

	private boolean m_open = false;

    // Opens claw
	// public Claw(PWM leftMotor, PWM rightMotor, CANSparkMax spinnyspin) {
	// 	m_leftMotor  = leftMotor;
	// 	m_rightMotor = rightMotor;
	// 	m_spinnyspin = spinnyspin;
		
	// 	setClaw(false);
	// }
	public Claw(Servo leftMotor, Servo rightMotor, CANSparkMax spinnyspin) {
		m_leftMotor  = leftMotor;
		m_rightMotor = rightMotor;
		m_spinnyspin = spinnyspin;
		
		// setClaw(false);
	}

	public void setAngle(double angle) {
		m_leftMotor.setAngle(angle);
		m_leftMotor.setAngle(180 - angle);
	}

    public void setClaw(boolean open) {
        // Open claw
		m_open = open;
		// m_leftMotor.setRaw(m_open ? 1000 : 2000);
		// m_rightMotor.setRaw(m_open ? 2000 : 1000);

		// m_leftMotor.setBounds(4000, 20000, 2000, 0, 0);

		// m_leftMotor.setAngle(m_open ? 0 : 180);
		// m_rightMotor.setAngle(m_open ? 180 : 0);
		m_leftMotor.setAngle(m_open ? 90 : 180);
		m_rightMotor.setAngle(m_open ? 90 : 0);

		// m_leftMotor.setRaw(m_open ? 0 : 4000);
		// m_rightMotor.setRaw(m_open ? 4000 : 0);

		// m_leftMotor.setRaw(m_open ? 1500 : 2000);
		// m_rightMotor.setRaw(m_open ? 1500 : 1000);

		// if (m_open)
		// 	m_spinnyspin.set(0.2);
		// else
		// 	m_spinnyspin.set(-0.2);	
		// new Timer().schedule(new TimerTask() {
		// 	@Override
		// 	public void run() {
		// 		nospinnyspin();
		// 	}
		// }, 750);
    }

	public void toggle() {
		setClaw(!m_open);
	}

	public boolean isClawOpen() {
		return m_open;
	}

	public void intake() {
		m_spinnyspin.set(0.2);
	}

	public void nospinnyspin() {
		m_spinnyspin.set(0);
	}

	public void outtake() {
		m_spinnyspin.set(-0.2);
	}

}
