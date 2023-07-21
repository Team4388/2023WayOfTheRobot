/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.lang.reflect.Field;
import java.lang.reflect.Member;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.Constants.ArmConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveModule;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
  private WPI_Pigeon2 m_pigeon2 = new WPI_Pigeon2(14);
  public RobotGyro gyro = new RobotGyro(m_pigeon2);


  public SwerveModule leftFront;
  public SwerveModule rightFront;
  public SwerveModule leftBack;
  public SwerveModule rightBack;

  public RobotMap() {
    configureLEDMotorControllers();
    configureDriveMotors();
    configArmMotors();

    try {
      setup_motor_tests();
    } catch (IllegalArgumentException | IllegalAccessException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /* LED Subsystem */
  // public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

  void configureLEDMotorControllers() {
        
  }

    // swerve drive subsystem
  public final WPI_TalonFX leftFrontWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_WHEEL_ID);
  public final WPI_TalonFX leftFrontSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_STEER_ID);
  public final CANCoder leftFrontEncoder = new CANCoder(SwerveDriveConstants.IDs.LEFT_FRONT_ENCODER_ID);

  public final WPI_TalonFX rightFrontWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_WHEEL_ID);
  public final WPI_TalonFX rightFrontSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_STEER_ID);
  public final CANCoder rightFrontEncoder = new CANCoder(SwerveDriveConstants.IDs.RIGHT_FRONT_ENCODER_ID);
    
  public final WPI_TalonFX leftBackWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_WHEEL_ID);
  public final WPI_TalonFX leftBackSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_STEER_ID);
  public final CANCoder leftBackEncoder = new CANCoder(SwerveDriveConstants.IDs.LEFT_BACK_ENCODER_ID);

  public final WPI_TalonFX rightBackWheel = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_WHEEL_ID);
  public final WPI_TalonFX rightBackSteer = new WPI_TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_STEER_ID);
  public final CANCoder rightBackEncoder = new CANCoder(SwerveDriveConstants.IDs.RIGHT_BACK_ENCODER_ID);

  void configureDriveMotors() {
    // config factory default
    leftFrontWheel.configFactoryDefault();
    leftFrontSteer.configFactoryDefault();
    
    rightFrontWheel.configFactoryDefault();
    rightFrontSteer.configFactoryDefault();
    
    leftBackWheel.configFactoryDefault();
    leftBackSteer.configFactoryDefault();
    
    rightBackWheel.configFactoryDefault();
    rightBackSteer.configFactoryDefault();

    // config open loop ramp
    leftFrontWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    leftFrontSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    
    rightFrontWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    rightFrontSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    
    leftBackWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    leftBackSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    
    rightBackWheel.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    rightBackSteer.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);

    // config closed loop ramp
    leftFrontWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    leftFrontSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    
    rightFrontWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    rightFrontSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    
    leftBackWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    leftBackSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    
    rightBackWheel.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    rightBackSteer.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);

    // config neutral deadband
    leftFrontSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
    leftFrontWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

    rightFrontWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
    rightFrontSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

    leftBackWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
    leftBackSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);

    rightBackWheel.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
    rightBackSteer.configNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND, SwerveDriveConstants.TIMEOUT_MS);
    
    // set neutral mode
    leftFrontWheel.setNeutralMode(NeutralMode.Brake);
    rightFrontWheel.setNeutralMode(NeutralMode.Brake);
    leftBackWheel.setNeutralMode(NeutralMode.Brake);
    rightBackWheel.setNeutralMode(NeutralMode.Brake);

    leftFrontSteer.setNeutralMode(NeutralMode.Brake);
    rightFrontSteer.setNeutralMode(NeutralMode.Brake);
    leftBackSteer.setNeutralMode(NeutralMode.Brake);
    rightBackSteer.setNeutralMode(NeutralMode.Brake);

    // initialize SwerveModules
    this.leftFront = new SwerveModule(leftFrontWheel, leftFrontSteer, leftFrontEncoder, -181.230469);
    this.rightFront = new SwerveModule(rightFrontWheel, rightFrontSteer, rightFrontEncoder, -270.615234);
    this.leftBack = new SwerveModule(leftBackWheel, leftBackSteer, leftBackEncoder, -240.029297);
    this.rightBack = new SwerveModule(rightBackWheel, rightBackSteer, rightBackEncoder, -40.869142);
  }

  // arm stuff
  public WPI_TalonFX pivot        = new WPI_TalonFX(15);
  public WPI_TalonFX tele         = new WPI_TalonFX(16);
  public CANCoder    pivotEncoder = new CANCoder(17);

  public void configArmMotors() {
    // config factory default
    pivot.configFactoryDefault();
    tele.configFactoryDefault();

    // config open loop ramp
    pivot.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    tele.configOpenloopRamp(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);

    // config closed loop ramp
    pivot.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);
    tele.configClosedloopRamp(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE, SwerveDriveConstants.TIMEOUT_MS);

    // config neutral mode to brake
    pivot.setNeutralMode(NeutralMode.Brake);
    tele.setNeutralMode(NeutralMode.Brake);

    // soft limits
    pivot.configForwardSoftLimitThreshold(ArmConstants.PIVOT_FORWARD_SOFT_LIMIT);
    pivot.configReverseSoftLimitThreshold(ArmConstants.PIVOT_REVERSE_SOFT_LIMIT);
    pivot.configForwardSoftLimitEnable(false);
    pivot.configReverseSoftLimitEnable(false);

    tele.configForwardSoftLimitThreshold(ArmConstants.TELE_FORWARD_SOFT_LIMIT);
    tele.configReverseSoftLimitThreshold(ArmConstants.TELE_REVERSE_SOFT_LIMIT);
    tele.configForwardSoftLimitEnable(false);
    tele.configReverseSoftLimitEnable(false);
  }

  // claw stuff (WHAT IS A SOAP ENGINEER)
  PWM         leftClaw   = new PWM(0);
  PWM         rightClaw  = new PWM(1);
  CANSparkMax spinnyspin = new CANSparkMax(18, MotorType.kBrushless);

  // ============================================
  // =============== AUTO TESTING ===============
  // ============================================

  static class MotorTest {
    boolean result;
    String  name;

    enum MotorType {FALCON, NEO}

    // why no union :(
    MotorType   motor_type;
    WPI_TalonFX as_falcon;
    CANSparkMax as_neo;
  }

  private static final long TEST_TIME = 500;

  public ArrayList<MotorTest> motor_tests = new ArrayList<>();
  private int    test_count = 0;
  private long   test_time  = 0;
  private double motor_position = 0;

  public void restart_motor_tests() {
    for (MotorTest test : motor_tests) {
      test.result = false;
    }

    test_time      = 0;
    test_count     = 0;
    motor_position = 0;
  }

  public boolean run_periodic_tests() {
    if (test_count >= motor_tests.size()) return true;

    MotorTest current_test = motor_tests.get(test_count);

    if (test_time == 0) {
      test_time = System.currentTimeMillis();

      switch (current_test.motor_type) {
        case FALCON:
          motor_position = current_test.as_falcon.getSelectedSensorPosition();
          current_test.as_falcon.set(.1);
          break;

        case NEO:
          // TODO: destroy robot
          break;
      }
    }

    if (System.currentTimeMillis() - test_time >= TEST_TIME) {
      switch (current_test.motor_type) {
        case FALCON:
        {
          double new_pos = current_test.as_falcon.getSelectedSensorPosition();
          if (Math.abs(new_pos - motor_position) > .1) {
            current_test.result = true;
          } else {
            System.out.printf("[DISCONNECTED] %s\n", current_test.name);
          }

          break;
        }

        case NEO: break;
      }

      test_time = 0;
      test_count++;
    }

    return false;
  }

  private void setup_motor_tests() throws IllegalArgumentException, IllegalAccessException {
    Class map_clazz = this.getClass();
    
    for (Field field : map_clazz.getFields()) {
      if (field.getType().equals(WPI_TalonFX.class)) {
        MotorTest test  = new MotorTest();
        test.result     = false;
        test.name       = field.getName();
        test.motor_type = MotorTest.MotorType.FALCON;
        test.as_falcon  = (WPI_TalonFX) field.get(this);
      }
    }
  }
}