/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc4388.robot.Constants.*;
import frc4388.robot.commands.AutoBalance;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.controller.DeadbandedXboxController;
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
    public final RobotMap m_robotMap = new RobotMap();

    /* Subsystems */
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront, m_robotMap.rightFront, m_robotMap.leftBack, m_robotMap.rightBack, m_robotMap.gyro);
    // private final LED m_robotLED = new LED(m_robotMap.LEDController);
    

    /* Controllers */
    // private final XboxController m_driverXbox = new XboxController(OIConstants.XBOX_DRIVER_ID);
    // private final XboxController m_operatorXbox = new XboxController(OIConstants.XBOX_OPERATOR_ID);

    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

    private boolean mode = true;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();

        /* Default Commands */

        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() ->
            m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(), getDeadbandedDriverController().getRight(), true)
          , m_robotSwerveDrive).withName("SwerveDrive DefaultCommand"));
        
        // m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetOdometry(), m_robotSwerveDrive));
        //     // .onFalse()

        new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
            .onTrue(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));

        new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.highSpeed(mode), m_robotSwerveDrive))
            .onFalse(new InstantCommand(() -> this.toggleMode()));
            
        // /* Operator Buttons */
        // // interrupt button
        // new JoystickButton(getOperatorJoystick(), XboxController.X_BUTTON)
        //     .onTrue(new InstantCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return new InstantCommand();
    }

    /**
     * Add your docs here.
     */
    // public IHandController getDriverController() {
    //     return m_driverXbox;
    // }

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

    public boolean getMode() {
        return this.mode;
    }

    public void toggleMode() {
        mode = !mode;
    }

    /**
     * Add your docs here.
     */
    // public IHandController getOperatorController() {
    //     return m_operatorXbox;
    // }

    /**
     * Add your docs here.
     */
    // public Joystick getOperatorJoystick() {
    //     return m_operatorXbox.getJoyStick();
    // }

    /**
     * Add your docs here.
     */
    // public Joystick getDriverJoystick() {
    //     return m_driverXbox.getJoyStick();
    // }
}
