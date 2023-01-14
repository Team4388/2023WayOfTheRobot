// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc4388.robot.Robot;
import frc4388.utility.RobotGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceTF2 extends PIDCommand {
  // private TalonSRX frontLeft;
  // private TalonSRX frontRight;
  // private TalonSRX backLeft;
  // private TalonSRX backRight;

  /** Creates a new AutoBalanceTF2. */
  public AutoBalanceTF2(TalonSRX frontLeft, TalonSRX frontRight, TalonSRX backLeft, TalonSRX backRight, RobotGyro gyro, Robot.MicroBot bot) {
    super(
        // The controller that the command will use
        new PIDController(.7, .02, .1),
        // This should return the measurement
        () -> Math.abs(gyro.getPitch()) < 3 ? 0 : gyro.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          double out2 = -MathUtil.clamp(output / 20, -1, 1);
          if (Math.abs(gyro.getPitch()) < 3) out2 = 0;

          SmartDashboard.putNumber("Output", out2);
          frontLeft.set(ControlMode.PercentOutput, out2);
          frontRight.set(ControlMode.PercentOutput, out2);
          backRight.set(ControlMode.PercentOutput, out2);
          backLeft.set(ControlMode.PercentOutput, out2);
        });

        gyro.reset();
        addRequirements(bot);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
