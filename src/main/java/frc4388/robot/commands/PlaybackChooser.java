package frc4388.robot.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4388.robot.subsystems.SwerveDrive;

public class PlaybackChooser {
    private final ArrayList<SendableChooser<Command>> m_choosers    = new ArrayList<>();
    private final SendableChooser<Command>            m_playback;
    private final HashMap<String, Command>            m_commandPool = new HashMap<>();

    private final File        m_dir = new File("/home/lvuser/autos/");
    private final SwerveDrive m_swerve;

    // commands
    private Command m_noAuto = new InstantCommand();
    
    public PlaybackChooser(SwerveDrive swerve, Object... pool) {
        m_swerve = swerve;

        for (int i = 0; i < pool.length; i += 2) {
            if (!(pool[i]     instanceof String) || !(pool[i + 1] instanceof Command)) {
                throw new RuntimeException("Need (string, command)");
            }

            m_commandPool.put((String) pool[i], (Command) pool[i + 1]);
        }

        appendCommand();
        m_playback = m_choosers.get(0);

        Shuffleboard.getTab("Auto Chooser")
            .add("Add Sequence", new InstantCommand(() -> appendCommand()))
            .withPosition(4, 0);
    }

    // This will be bound to a button for the time being
    public void appendCommand() {
        var chooser = new SendableChooser<Command>();

        for (String auto : m_dir.list()) {
            m_playback.addOption(auto, new JoystickPlayback(m_swerve, auto));
        }
        for (var cmd_name : m_commandPool.keySet()) {
            chooser.addOption(cmd_name, m_commandPool.get(cmd_name));
        }
        chooser.addOption("No Auto", m_noAuto);

        m_choosers.add(chooser);
        Shuffleboard.getTab("Auto Chooser")
            .add("Command: " + m_choosers.size(), chooser)
            .withSize(4, 1)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);
    }

    public Command getCommand() {
        Command command = m_playback.getSelected();
        command = command == null ? m_noAuto : command.asProxy();
        
        Command[] commands = new Command[m_choosers.size() - 1];
        for (int i = 0; i < m_choosers.size()-1; i++) {
            Command command2 = m_choosers.get(i + 1).getSelected();
            command2 = command2 == null ? m_noAuto : command2.asProxy();

            commands[i] = command2.asProxy();
        }

        return command.andThen(commands);
    }
}
