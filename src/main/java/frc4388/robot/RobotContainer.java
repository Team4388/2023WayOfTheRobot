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
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.FaultID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc4388.robot.Constants.*;
import frc4388.robot.subsystems.Arm;
import frc4388.robot.subsystems.Claw;
import frc4388.robot.subsystems.Limelight;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.commands.BooleanCommand;
import frc4388.robot.commands.TimedCommand;
import frc4388.robot.commands.Arm.PivotCommand;
import frc4388.robot.commands.Arm.TeleCommand;
import frc4388.robot.commands.Autos.AutoBalance;
import frc4388.robot.commands.Autos.PlaybackChooser;
import frc4388.robot.commands.Placement.AprilRotAlign;
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

    public final Claw m_robotClaw = new Claw(m_robotMap.leftClaw, m_robotMap.rightClaw, m_robotMap.spinnyspin);

    public final Limelight m_robotLimeLight = new Limelight();

    /* Controllers */
    private final DeadbandedXboxController m_driverXbox = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);

    private PlaybackChooser playbackChooser;

    /* Commands */
    private Command emptyCommand = new InstantCommand();
    private Command interruptCommand = new InstantCommand(() -> {}, m_robotArm, m_robotSwerveDrive, m_robotClaw, m_robotLimeLight);

    private SequentialCommandGroup armToHome = new SequentialCommandGroup(new TeleCommand(m_robotArm, 1000), new PivotCommand(m_robotArm, 145));

    private Command toggleClaw = new InstantCommand(() -> m_robotClaw.toggle(), m_robotClaw);

    private Command toggleClawCones = new InstantCommand(() -> m_robotClaw.toggleCones(), m_robotClaw);
    private Command toggleClawCubes = new InstantCommand(() -> m_robotClaw.toggleCubes(), m_robotClaw);

    public boolean readyForPlacement = true;
    private Boolean isPole = null;

    private SequentialCommandGroup alignToPole =
        new SequentialCommandGroup(
            new InstantCommand(() -> m_robotLimeLight.setDriverMode(false), m_robotLimeLight),
            new InstantCommand(() -> m_robotLimeLight.setToLimePipeline(), m_robotLimeLight),
            new RotateToAngle(m_robotSwerveDrive, 0.0),
            new LimeAlign(m_robotSwerveDrive, m_robotLimeLight, () -> m_robotLimeLight.getLowestTape().getYaw(), 0.04),
            new RotateToAngle(m_robotSwerveDrive, 0.0),
            new RunCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0.0, -0.4), new Translation2d(0.0, 0.0), true), m_robotSwerveDrive)
        ).andThen(new InstantCommand(() -> readyForPlacement = true), new InstantCommand(() -> isPole = true));
    
    private SequentialCommandGroup alignToShelf = 
        new SequentialCommandGroup(
            new InstantCommand(() -> m_robotLimeLight.setDriverMode(false), m_robotLimeLight),
            new InstantCommand(() -> m_robotLimeLight.setToAprilPipeline(), m_robotLimeLight),
            new AprilRotAlign(m_robotSwerveDrive, m_robotLimeLight),
            new LimeAlign(m_robotSwerveDrive, m_robotLimeLight, () -> m_robotLimeLight.getAprilPoint().getYaw(), 0.04),
            new AprilRotAlign(m_robotSwerveDrive, m_robotLimeLight),
            new RunCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0.0, -0.4), new Translation2d(0.0, 0.0), true), m_robotSwerveDrive)
        ).andThen(new InstantCommand(() -> readyForPlacement = true), new InstantCommand(() -> isPole = false));

    public SequentialCommandGroup place = null;

    private Consumer<SequentialCommandGroup> queuePlacement = (scg) -> {
        place = scg.asProxy().andThen(new InstantCommand(() -> m_robotLimeLight.setDriverMode(true), m_robotLimeLight), new InstantCommand(() -> readyForPlacement = true), new InstantCommand(() -> isPole = null), new InstantCommand(() -> place = null));
    };

    private SequentialCommandGroup placeCubeHigh =
        new SequentialCommandGroup(
            new PivotCommand(m_robotArm, 60 + 135),
            new WaitCommand(0.3),
            new TeleCommand(m_robotArm, 95000)
            // toggleClaw.asProxy(),
            // armToHome.asProxy()
        );

    private SequentialCommandGroup placeCubeMid = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 55 + 135),
        new WaitCommand(0.3),
        new TeleCommand(m_robotArm, 28000)
        // toggleClaw.asProxy(),
        // armToHome.asProxy()
    );

    private SequentialCommandGroup placeCubeLow = new SequentialCommandGroup(
        new TimedCommand(new RunCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0.0, -0.2), new Translation2d(0.0, 0.0), true), m_robotSwerveDrive), 0.7),
        new PivotCommand(m_robotArm, 70 + 135),
        new WaitCommand(0.3),
        new TeleCommand(m_robotArm, 28000)
        // toggleClaw.asProxy(),
        // armToHome.asProxy()
    );

    private SequentialCommandGroup placeConeHigh = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 178),
        new WaitCommand(0.3),
        new TeleCommand(m_robotArm, 56000)
        // new ParallelRaceGroup(new InstantCommand(() -> m_robotClaw.toggle()), new RunCommand(() -> m_robotClaw.outtake()))
        // toggleClaw.asProxy(),
        // armToHome.asProxy()
    );

    private SequentialCommandGroup placeConeMid = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 135 + 47),
        new WaitCommand(0.3),
        new TeleCommand(m_robotArm, 30000)
        // toggleClaw.asProxy(),
        // armToHome.asProxy()
    );

    private SequentialCommandGroup placeLow = new SequentialCommandGroup(
        new PivotCommand(m_robotArm, 0),
        new TeleCommand(m_robotArm, 0),
        toggleClaw.asProxy(),
        armToHome.asProxy()
    );

    /* Autos */
    public SendableChooser<Command> chooser = new SendableChooser<>();

    // private Command taxi = new JoystickPlayback(m_robotSwerveDrive, "Taxi.txt");

    private Command wait3 = new WaitCommand(3);
    private Command wait5 = new WaitCommand(5);


    private SequentialCommandGroup taxiFar = new SequentialCommandGroup(
        placeConeMid.asProxy(),
        new WaitCommand(0.3),
        toggleClaw.asProxy(),
        new WaitCommand(0.2),
        toggleClaw.asProxy(),
        new WaitCommand(0.3),
        // new ParallelCommandGroup(
            armToHome.asProxy()
            // new TimedCommand(new RunCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0.0, 0.6), new Translation2d(0.0, 0.0), true), m_robotSwerveDrive), 1.0)
        // )
    );

    private SequentialCommandGroup placeRed1Balance = new SequentialCommandGroup(
        placeConeMid.asProxy(),
        new WaitCommand(0.3),
        toggleClaw.asProxy(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            armToHome.asProxy(),
            new JoystickPlayback(m_robotSwerveDrive, "idk", 1)
        ),
        new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive)
    );

    private SequentialCommandGroup placeTaxi = new SequentialCommandGroup(
        placeConeMid.asProxy(),
        new WaitCommand(0.3),
        // new InstantCommand(() -> m_robotClaw.setClaw(true), m_robotClaw),
        toggleClaw.asProxy(),
        new WaitCommand(0.2),
        toggleClaw.asProxy(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            armToHome.asProxy(),
            new TimedCommand(new RunCommand(() -> m_robotSwerveDrive.driveWithInput(new Translation2d(0.0, 0.6), new Translation2d(0.0, 0.0), true), m_robotSwerveDrive), 4.9)
        )
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
        chooser.setDefaultOption("NoAuto", emptyCommand);
        chooser.addOption("alignToPole", alignToPole);
        chooser.addOption("alignToShelf", alignToShelf);

        chooser.addOption("placeConeHigh", placeConeHigh);
        chooser.addOption("placeConeMid", placeConeMid);
        chooser.addOption("placeCubeHigh", placeCubeHigh);
        chooser.addOption("placeCubeMid", placeCubeMid);
        chooser.addOption("placeLow", placeLow);

        playbackChooser = new PlaybackChooser(m_robotSwerveDrive)
            .addOption("PlaceTaxi", placeTaxi)
            .addOption("PlaceRed1Balance", placeRed1Balance)
            .addOption("Wait3", wait3.asProxy())
            .addOption("Wait5", wait5.asProxy())
            .addOption("TaxiFar", taxiFar.asProxy())
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

        // * TEST BUTTONS

        // * Driver Buttons
        new JoystickButton(getDeadbandedDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro(), m_robotSwerveDrive)); // final
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.setToTurbo()))
            .onFalse(new InstantCommand(() -> m_robotSwerveDrive.setToFast()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.setToSlow()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
            .onTrue(new AutoBalance(m_robotMap.gyro, m_robotSwerveDrive)); // final

        new JoystickButton(getDeadbandedDriverController(), XboxController.X_BUTTON)
            .onTrue(interruptCommand.asProxy()); // final

        // new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_TRIGGER_AXIS)
        //     .whileTrue(new JoystickRecorder(m_robotSwerveDrive,
        //                                     () -> getDeadbandedDriverController().getLeftX(),
        //                                     () -> getDeadbandedDriverController().getLeftY(),
        //                                     () -> getDeadbandedDriverController().getRightX(),
        //                                     () -> getDeadbandedDriverController().getRightY(),
        //                                     "Red1Balance.txt"))
        //     .onFalse(new InstantCommand());

        // new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON)
        //     .onTrue(new JoystickPlayback(m_robotSwerveDrive, "Red1Balance.txt"))
        //     .onFalse(new InstantCommand()); 

        // * Operator Buttons
        
        // align (pole)
        new JoystickButton(getDeadbandedOperatorController(), XboxController.B_BUTTON) // final
            .onTrue(alignToPole)
            .onFalse(interruptCommand.asProxy());
        
        // align (shelf)
        new JoystickButton(getDeadbandedOperatorController(), XboxController.A_BUTTON) // final
            .onTrue(alignToShelf)
            .onFalse(interruptCommand.asProxy());

        // toggle claw
        new JoystickButton(getDeadbandedOperatorController(), XboxController.X_BUTTON) // final
            .onTrue(toggleClaw.asProxy());

        // new JoystickButton(getDeadbandedOperatorController(), XboxController.X_BUTTON) // final
        //     .onTrue(new ConditionalCommand(toggleClawCones.asProxy(), toggleClawCubes.asProxy(), () -> SmartDashboard.getBoolean("Cones?", true)));
        
        // kill soft limits
        new JoystickButton(getDeadbandedOperatorController(), XboxController.Y_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotArm.killSoftLimits()));

        // outtake
        new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .whileTrue  (new RunCommand(() -> m_robotClaw.outtake()))
            .onFalse (new InstantCommand(() -> m_robotClaw.nospinnyspin()));
        
        // intake    
        new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .whileTrue  (new RunCommand(() -> m_robotClaw.intake()))
            .onFalse (new InstantCommand(() -> m_robotClaw.nospinnyspin()));
        
        // arm to Home
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.LEFT_BUMPER_BUTTON) // final
        //     .onTrue(armToHome.asProxy());

        // interupt
        // new JoystickButton(getDeadbandedOperatorController(), XboxController.RIGHT_BUMPER_BUTTON) // final
        //     .onTrue(interruptCommand.asProxy());


        // // place high
        // new POVButton(getDeadbandedOperatorController(), 0)
        //     .onTrue(new ConditionalCommand(
        //         new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeConeHigh)), new InstantCommand(() -> queuePlacement.accept(placeCubeHigh)), () -> isPole == true),
        //         emptyCommand.asProxy(),
        //         () -> readyForPlacement == true)
            // );
        
        // place high
        new POVButton(getDeadbandedOperatorController(), 0)
            .onTrue(new ConditionalCommand(
                new InstantCommand(() -> queuePlacement.accept(placeConeHigh)),
                emptyCommand.asProxy(),
                () -> readyForPlacement == true)
            );

        // place mid
        new POVButton(getDeadbandedOperatorController(), 270)
            .onTrue(new ConditionalCommand(
                new InstantCommand(() -> queuePlacement.accept(placeConeMid)),
                emptyCommand.asProxy(),
                () -> readyForPlacement == true)
            );

        // // place mid
        // new POVButton(getDeadbandedOperatorController(), 270)
        //     .onTrue(new ConditionalCommand(
        //         new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeConeMid)), new InstantCommand(() -> queuePlacement.accept(placeCubeMid)), () -> isPole == true),
        //         emptyCommand.asProxy(),
        //         () -> readyForPlacement == true)
        //     );

        // // place low
        // new POVButton(getDeadbandedOperatorController(), 180)
        //     .onTrue(new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeLow)), emptyCommand.asProxy(), () -> readyForPlacement == true));
        
        // place low
        new POVButton(getDeadbandedOperatorController(), 180)
            .onTrue(new ConditionalCommand(new InstantCommand(() -> queuePlacement.accept(placeCubeLow)), emptyCommand.asProxy(), () -> readyForPlacement == true));

        // confirm
        new POVButton(getDeadbandedOperatorController(), 90)
            .onTrue(new BooleanCommand(() -> place, () -> emptyCommand.asProxy(), () -> place != null));

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
