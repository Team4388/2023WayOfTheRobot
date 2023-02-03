/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.commands.AutoBalance;
import frc4388.robot.commands.DriveWithInput;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.LEDPatterns;
import frc4388.utility.RobotGyro;
import frc4388.utility.controller.IHandController;
import frc4388.utility.controller.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* RobotMap */
    private final RobotMap m_robotMap = new RobotMap();

    /* Subsystems */
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront, m_robotMap.rightFront, m_robotMap.leftBack, m_robotMap.rightBack);//, m_robotMap.gyro);
    // private final LED m_robotLED = new LED(m_robotMap.LEDController);
    

    /* Controllers */
    private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
    private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);

    // public RobotGyro gyroRef = m_robotMap.gyro;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();

        m_robotSwerveDrive.setDefaultCommand(new DriveWithInput(m_robotSwerveDrive, 
                                                                () -> getDriverController().getLeftXAxis(), 
                                                                () -> getDriverController().getLeftYAxis(), 
                                                                () -> getDriverController().getRightXAxis(),
                                                                false));

        /* Default Commands */
        // drives the robot with a two-axis input from the driver controller
        // continually sends updates to the Blinkin LED controller to keep the lights on
        // m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));

        // m_robotSwerveDrive.setDefaultCommand(
        //     new RunCommand(() -> m_robotSwerveDrive.driveWithInput(-0.3*getDriverController().getLeftXAxis(), 
        //                                                             0.3*getDriverController().getLeftYAxis(), 
        //                                                            -0.3*getDriverController().getRightXAxis(), false), m_robotSwerveDrive)
        // );

        // m_robotSwerveDrive.setDefaultCommand(
        //     new RunCommand(() -> m_robotSwerveDrive.driveWithInput(0, 
        //                                                             -0.1, 
        //                                                            0, false), m_robotSwerveDrive)
        // );

    //     m_robotSwerveDrive.setDefaultCommand(
    //         new RunCommand(() -> m_robotSwerveDrive.driveWithInput(
    //             -0.3 * getDriverController().getLeftXAxis(),
    //             0.3 * getDriverController().getLeftYAxis(),
    //             0.3 * getDriverController().getRightXAxis(),
    //             0.3 * getDriverController().getRightYAxis(),
    //             true),
    //             m_robotSwerveDrive).withName("Swerve driveWithInput defaultCommand"));
    
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        
        // new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)
        //     .onTrue(new InstantCommand(() -> gyroRef.reset()));

        /* Operator Buttons */
        // activates "Lit Mode"
        // new JoystickButton(getOperatorJoystick(), XboxController.A_BUTTON)
        //     .whileTrue(new RunCommand(() -> m_robotLED.setPattern(LEDPatterns.LAVA_RAINBOW), m_robotLED))
        //     .whileFalse(new RunCommand(() -> m_robotLED.setPattern(LEDConstants.DEFAULT_PATTERN), m_robotLED));

        // new JoystickButton(getDriverJoystick(), XboxController.A_BUTTON)
        //     .whileTrue(new RunCommand(() -> m_robotSwerveDrive.rotateCANCodersToAngle(0), m_robotSwerveDrive));
        
        // new JoystickButton(getDriverJoystick(), XboxController.B_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetCANCoders(0), m_robotSwerveDrive));

        // new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
        //     .onTrue()
        new JoystickButton(getDriverJoystick(), XboxController.Y_BUTTON)
            .onTrue(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));

        //New interupt button
        new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // no auto
        // TrajectoryConfig trajConfig = new TrajectoryConfig(1, 1).setKinematics(m_robotSwerveDrive.getKinematics()); // TODO: make these AutoConstants

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)), null, null, trajConfig);

        return new InstantCommand();
    }

    /**
     * Add your docs here.
     */
    public IHandController getDriverController() {
        return m_driverXbox;
    }

    /**
     * Add your docs here.
     */
    public IHandController getOperatorController() {
        return m_operatorXbox;
    }

    /**
     * Add your docs here.
     */
    public Joystick getOperatorJoystick() {
        return m_operatorXbox.getJoyStick();
    }

    /**
     * Add your docs here.
     */
    public Joystick getDriverJoystick() {
        return m_driverXbox.getJoyStick();
    }
}
