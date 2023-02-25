/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.objdetect.HOGDescriptor;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.Constants.SwerveDriveConstants.AutoConstants;
import frc4388.robot.Constants.SwerveDriveConstants.PIDConstants;
import frc4388.robot.commands.AutoBalance;
import frc4388.robot.commands.JoystickPlayback;
import frc4388.robot.commands.JoystickRecorder;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.controller.DeadbandedXboxController;
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
    public final RobotMap m_robotMap = new RobotMap();

    /* Subsystems */
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                                                  m_robotMap.gyro);

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

    /* Autos */
    private SendableChooser<Command> chooser = new SendableChooser<>();
    
    private Command noAuto = new InstantCommand();
    
    private Command balance = new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive);
    
    private Command blue1Path = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt");
    private Command blue1PathWithBalance = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt").andThen(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));
    
    private Command red1Path = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt", -1);
    private Command red1PathWithBalance = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt", -1).andThen(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));

    private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();

        // * Default Commands
        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
                m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(),
                                                  getDeadbandedDriverController().getRight(),
                                                  true);
            }, m_robotSwerveDrive)
            .withName("SwerveDrive DefaultCommand"));
        
        // * Auto Commands
        chooser.setDefaultOption("NoAuto", noAuto);

        chooser.addOption("Blue1Path", blue1Path);
        chooser.addOption("Blue1PathWithBalance", blue1PathWithBalance);

        chooser.addOption("Red1Path", red1Path);
        chooser.addOption("Red1PathWithBalance", red1PathWithBalance);
        
        chooser.addOption("Taxi", taxi);

        SmartDashboard.putData(chooser);
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // * Driver Buttons
        new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive));
        
        // new JoystickButton(getDeadbandedDriverController(), XboxController.X_BUTTON)
        //     .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(), m_robotSwerveDrive));
        //     // .onFalse()

        new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
            .onTrue(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));

        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
            .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
                                            () -> getDeadbandedDriverController().getLeftX(),
                                            () -> getDeadbandedDriverController().getLeftY(),
                                            () -> getDeadbandedDriverController().getRightX(),
                                            () -> getDeadbandedDriverController().getRightY(),
                                            "Blue1Path.txt"))
            .onFalse(new InstantCommand());

        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt"));

        // * Operator Buttons
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return chooser.getSelected();
        // return new InstantCommand();
    }

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

}
