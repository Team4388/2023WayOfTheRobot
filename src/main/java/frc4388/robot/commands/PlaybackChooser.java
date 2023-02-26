package frc4388.robot.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4388.robot.subsystems.SwerveDrive;

public class PlaybackChooser {
    private ArrayList<SendableChooser<Command>> m_choosers    = new ArrayList<>();
    private SendableChooser<Command>            m_playback    = new SendableChooser<>();
    private HashMap<String, Command>            m_commandPool = new HashMap<>();

    private File        m_dir = new File("/home/lvuser/autos/");
    private SwerveDrive m_swerve;

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

        for (String auto : m_dir.list()) {
            m_playback.addOption(auto, new JoystickPlayback(m_swerve, auto));
        }
        m_playback.addOption("No Auto", m_noAuto);

        m_choosers.add(m_playback);
        SmartDashboard.putData("Command: 0", m_playback);
    }

    // This will be bound to a button for the time being
    public void appendCommand() {
        var chooser = new SendableChooser<Command>();

        for (var cmd_name : m_commandPool.keySet()) {
            chooser.addOption(cmd_name, m_commandPool.get(cmd_name));
        }

        m_choosers.add(chooser);
        SmartDashboard.putData("Command: " + m_choosers.size(), chooser);
    }

    // This will be bound to a button for the time being
    public void appendPlayback() {
        var chooser = new SendableChooser<Command>();

        for (String auto : m_dir.list()) {
            m_playback.addOption(auto, new JoystickPlayback(m_swerve, auto));
        }

        m_choosers.add(chooser);
        SmartDashboard.putData("Command: " + m_choosers.size(), chooser);
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
