package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class MotorTest extends SubsystemBase {
  /** Creates a new MotorTest. */
  WPI_TalonFX motor1;
  WPI_TalonFX motor2;
  public MotorTest(WPI_TalonFX motor1, WPI_TalonFX motor2) {
    this.motor1 = motor1;
    this.motor2 = motor2;
  }
  

  public double motorSetSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
    return speed;
  }
  public void stop() {
        motor1.set(0);
        motor2.set(0);
  }


}