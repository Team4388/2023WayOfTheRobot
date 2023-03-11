/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.lang.System;
import java.lang.reflect.Array;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.utility.RobotTime;

import frc4388.robot.subsystems.Location;
import frc4388.robot.subsystems.Apriltags.Tag;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Command m_autonomousCommand;
  
  private RobotTime m_robotTime = RobotTime.getInstance();
  private RobotContainer m_robotContainer;

  private Location location = new Location();


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("AutoPlayback Chooser", m_robotContainer.chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_robotTime.updateTimes();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    final Tag pos = location.getPosRot();
    if (pos != null) {
      SmartDashboard.putNumber("x position", pos.x);
    }

    //ystem.out.print(apriltagPos[0]);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  boolean dis = false;
  @Override
  public void disabledInit() {
    // m_robotContainer.m_robotClaw.setClaw(false);
    m_robotTime.endMatchTime();
    m_robotContainer.m_robotClaw.disable();

    m_handle = m_robotContainer.m_robotMap.leftBackWheel.getHandle();
  }

  long m_handle = 0;
  @Override
  public void disabledPeriodic() {
    // if (dis) {
      
    //   MotControllerJNI.Set_4(m_handle, ControlMode.PercentOutput.value, 1, 1, DemandType.Neutral.value);
    //   // m_robotContainer.m_robotMap.leftBackSteer.set(ControlMode.PercentOutput, 1, DemandType.Neutral, 0);
    //   // m_robotContainer.m_robotSwerveDrive.driveWithInput(new Translation2d(.5, 0), new Translation2d(), true);
    // }

    // System.out.println("hi from disabled");
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_robotSwerveDrive.resetGyro();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.m_robotSwerveDrive.resetGyro();
    dis = true;

    m_robotContainer.m_robotClaw.enable();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_robotSwerveDrive.resetGyro();
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
