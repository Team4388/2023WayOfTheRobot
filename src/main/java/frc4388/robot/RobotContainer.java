/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.FaultID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.Arm;
import frc4388.robot.subsystems.MotorTest;
import frc4388.robot.subsystems.Claw;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.commands.BooleanCommand;
import frc4388.robot.commands.Arm.PivotCommand;
import frc4388.robot.commands.Arm.TeleCommand;
import frc4388.robot.commands.Autos.AutoBalance;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Placement.AprilRotAlign;
import frc4388.robot.commands.Placement.LimeAlign;
import frc4388.robot.commands.Swerve.RotateToAngle;
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

    
    public final Arm m_robotArm = new Arm(m_robotMap.pivot, m_robotMap.tele, m_robotMap.pivotEncoder);

    public final Claw m_robotClaw = new Claw(m_robotMap.leftClaw, m_robotMap.rightClaw, m_robotMap.spinnyspin);

    public final Limelight m_robotLimeLight = new Limelight();

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

    /* Autos */
    public SendableChooser<Command> chooser = new SendableChooser<>();
    // TODO:  PUT PARAMETERS FOR MOTORS
    MotorTest m_motortest = new MotorTest(new WPI_TalonFX(1), new WPI_TalonFX(1));
    private PlaybackChooser playbackChooser;

    /* Commands */
    private Command emptyCommand = new InstantCommand();
   
    private SequentialCommandGroup armToHome = new SequentialCommandGroup(new TeleCommand(m_robotArm, 0), new PivotCommand(m_robotArm, 135));

    private Command toggleClaw = new InstantCommand(() -> m_robotClaw.toggle(), m_robotClaw);

    private boolean readyForPlacement = false;
    private Boolean isPole = null;





    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();


        m_motortest.setDefaultCommand(new RunCommand(() -> {
            m_motortest.motorSetSpeed(getDeadbandedDriverController().getLeftX());
        }, m_robotArm)
        .withName("Arm DefaultCommand"));
        
        // * Auto Commands


        // chooser.addOption("Blue1Path", blue1Path);
        // chooser.addOption("Blue1PathWithBalance", blue1PathWithBalance);

        // chooser.addOption("Red1Path", red1Path);
        // chooser.addOption("Red1PathWithBalance", red1PathWithBalance);

 
    }

    // here be dragons - enter if you dare
    private static class TapState {
        public boolean gearTapped = true;
        public long    gearTime   = 0;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // * Driver Buttons
      // * Driver Buttons
       
            // .onTrue(new InstantCommand(() -> {
            //     tap.gearTapped = true;
            //     tap.gearTime   = System.currentTimeMillis();
            // }))
            // .whileTrue(new RunCommand(() -> {
            //     if (tap.gearTapped && System.currentTimeMillis() - tap.gearTime > 200) {
            //         m_robotSwerveDrive.setToTurbo();
            //         tap.gearTapped = false;
            //     }
            // }))
            // .onFalse(new InstantCommand(() -> {
            //     if (tap.gearTapped)
            //         m_robotSwerveDrive.setToFast();
            //     else
            //         m_robotSwerveDrive.setToSlow();
            // }));
        
 


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return chooser.getSelected();
        return playbackChooser.getCommand();
    }

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

}
