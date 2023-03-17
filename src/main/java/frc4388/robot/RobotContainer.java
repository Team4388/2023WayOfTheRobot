/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.function.Consumer;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.Arm;
import frc4388.robot.subsystems.Claw;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.commands.Arm.PivotCommand;
import frc4388.robot.commands.Arm.TeleCommand;
import frc4388.robot.commands.Autos.AutoBalance;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Placement.DriveToLimeDistance;
import frc4388.robot.commands.Placement.LimeAlign;
import frc4388.robot.commands.Swerve.JoystickPlayback;
import frc4388.robot.commands.Swerve.JoystickRecorder;
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
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.leftFront,
                                                                  m_robotMap.rightFront,
                                                                  m_robotMap.leftBack,
                                                                  m_robotMap.rightBack,
                                                                  m_robotMap.gyro);
    
    public final Arm m_robotArm = new Arm(m_robotMap.pivot, m_robotMap.tele, m_robotMap.pivotEncoder);

    public final Claw m_robotClaw = new Claw(m_robotMap.servo);

    public final Limelight m_robotLimeLight = new Limelight();

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

    /* Autos */
    public SendableChooser<Command> chooser = new SendableChooser<>();

    // private Command balance = new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive);
    
    // private Command blue1Path = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt");
    // private Command blue1PathWithBalance = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt").andThen(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));
    
    // private Command red1Path = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt", -1);
    // private Command red1PathWithBalance = new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt", -1).andThen(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));

    // private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");

    private PlaybackChooser playbackChooser;

    /* Commands */
    private Command emptyCommand = new InstantCommand();

    private SequentialCommandGroup armToHome = new SequentialCommandGroup(new TeleCommand(m_robotArm, 0), new PivotCommand(m_robotArm, 0));

    private Command toggleClaw = new InstantCommand(() -> m_robotClaw.toggle(), m_robotClaw);

    private boolean readyForPlacement = false;
    private Boolean isPole = null;

    private ConditionalCommand alignToPole = new ConditionalCommand(
        new SequentialCommandGroup(
            new RotateToAngle(m_robotSwerveDrive, 0.0),
            new LimeAlign(m_robotSwerveDrive, m_robotLimeLight),
            new DriveToLimeDistance(m_robotSwerveDrive, m_robotLimeLight, 30)
        ).andThen(new InstantCommand(() -> readyForPlacement = true), new InstantCommand(() -> isPole = true)),
        emptyCommand.asProxy(),
        () -> m_robotLimeLight.numTargets() <= 2
    );

    // TODO: make
    private ConditionalCommand alignToShelf = new ConditionalCommand(
        new SequentialCommandGroup(

        ).andThen(new InstantCommand(() -> readyForPlacement = true), new InstantCommand(() -> isPole = false)),
        emptyCommand.asProxy(),
        () -> true
    );

    public SequentialCommandGroup place = null;

    private Consumer<SequentialCommandGroup> queuePlacement = (scg) -> {
        place = scg.andThen(new InstantCommand(() -> readyForPlacement = false), new InstantCommand(() -> isPole = null));
    };

    // TODO: find actual values
    private SequentialCommandGroup placeConeHigh = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 0),
        new TeleCommand(m_robotArm, 0),
        toggleClaw.asProxy(),
        armToHome.asProxy()
    );

    private SequentialCommandGroup placeConeMid = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 0),
        new TeleCommand(m_robotArm, 0),
        toggleClaw.asProxy(),
        armToHome.asProxy()
    );

    private SequentialCommandGroup placeCubeHigh = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 0),
        new TeleCommand(m_robotArm, 0),
        toggleClaw.asProxy(),
        armToHome.asProxy()
    );

    private SequentialCommandGroup placeCubeMid = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 0),
        new TeleCommand(m_robotArm, 0),
        toggleClaw.asProxy(),
        armToHome.asProxy()
    );

    private SequentialCommandGroup placeLow = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 0),
        new TeleCommand(m_robotArm, 0),
        toggleClaw.asProxy(),
        armToHome.asProxy()
    );


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

        m_robotArm.setDefaultCommand(new RunCommand(() -> {
            m_robotArm.setRotVel(getDeadbandedOperatorController().getLeftY());
            m_robotArm.setTeleVel(getDeadbandedOperatorController().getRightY());
        }, m_robotArm)
        .withName("Arm DefaultCommand"));
        
        // * Auto Commands
        // chooser.setDefaultOption("NoAuto", noAuto);

        // chooser.addOption("Blue1Path", blue1Path);
        // chooser.addOption("Blue1PathWithBalance", blue1PathWithBalance);

        // chooser.addOption("Red1Path", red1Path);
        // chooser.addOption("Red1PathWithBalance", red1PathWithBalance);
        
        // chooser.addOption("Taxi", taxi);

        playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
            .addOption("Balance", new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive))
            .buildDisplay();
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
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.setToTurbo()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.setToSlow()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
            .onTrue(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive));

        // new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON)
        //     .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
        //                                     () -> getDeadbandedDriverController().getLeftX(),
        //                                     () -> getDeadbandedDriverController().getLeftY(),
        //                                     () -> getDeadbandedDriverController().getRightX(),
        //                                     () -> getDeadbandedDriverController().getRightY(),
        //                                     "Blue1Path.txt"))
        //     .onFalse(new InstantCommand());

        // new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
        //     .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Blue1Path.txt"))
        //     .onFalse(new InstantCommand());

        // * Operator Buttons

        
        // align (pole)
        new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON)
            .onTrue(alignToPole);
        
        // align (shelf)
        new JoystickButton(getDeadbandedOperatorController(), XboxController.B_BUTTON)
            .onTrue(alignToShelf);

        // toggle claw
        new JoystickButton(getDeadbandedOperatorController(), XboxController.X_BUTTON)
            .onTrue(toggleClaw.asProxy());

        // kill soft limits
        new JoystickButton(getDeadbandedOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotArm.killSoftLimits()));
        
        // toggle limelight
        new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotLimeLight.toggleLEDs(), m_robotLimeLight));
        
        // interrupt button
        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON)
            .onTrue(new InstantCommand(() -> {}, m_robotArm, m_robotSwerveDrive, m_robotClaw, m_robotLimeLight));

        // place high
        new POVButton(getDeadbandedOperatorController(), 0)
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeConeHigh)), new InstantCommand(() -> queuePlacement.accept(placeCubeHigh)), () -> isPole == true),
                emptyCommand.asProxy(),
                () -> readyForPlacement == true)
            );

        // place mid
        new POVButton(getDeadbandedOperatorController(), 270)
            .onTrue(new ConditionalCommand(
                new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeConeMid)), new InstantCommand(() -> queuePlacement.accept(placeCubeMid)), () -> isPole == true),
                emptyCommand.asProxy(),
                () -> readyForPlacement == true)
            );

        // place low
        new POVButton(getDeadbandedOperatorController(), 180)
            .onTrue(new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeLow)), emptyCommand.asProxy(), () -> readyForPlacement == true));

        // confirm
        new POVButton(getDeadbandedOperatorController(), 90)
            .onTrue(new ConditionalCommand(place, emptyCommand.asProxy(), () -> place != null));

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
