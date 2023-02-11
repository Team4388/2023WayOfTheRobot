// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.Time;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.controller.DeadbandedXboxController;

public class JoystickPlayback extends CommandBase {
  private static class TimedOutput {
    public double leftX  = 0d;
    public double leftY  = 0d;
    public double rightX = 0d;
    public double rightY = 0d;

    public long timed_offset = 0l;
  }

  private final SwerveDrive            m_swerve;
  private       Scanner                m_input;
  private final ArrayList<TimedOutput> m_outputs;
  private       long                   m_playback_time;
  private       int                    m_last_index;
  private       boolean                m_finished = false; // ! find a better way

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerve = swerve;
    m_outputs = new ArrayList<>();

    try {
      m_input = new Scanner(new File("/home/lvuser/JoystickInputs.txt"));

      String line = "";
      while (m_input.hasNextLine()) {
        line = m_input.nextLine();
      
        String[] values = line.split(",");

        var out = new TimedOutput();
        out.leftX  = Double.parseDouble(values[0]);
        out.leftY  = Double.parseDouble(values[1]);
        out.rightX = Double.parseDouble(values[2]);
        out.rightY = Double.parseDouble(values[3]);

        out.timed_offset = Long.MAX_VALUE;

        m_outputs.add(out);
      }
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    addRequirements(this.m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("STARTING PLAYBACK");
    System.out.println("STARTING PLAYBACK");
    System.out.println("STARTING PLAYBACK");
    System.out.println("STARTING PLAYBACK");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // skip to reasonable time frame
    // too tired to write comment: ask daniel thomas; it goes to the thing until it's bigger than the other thing
    {
      int i = m_last_index + 1;
      while (i < m_outputs.size() && m_outputs.get(i).timed_offset < m_playback_time) {
        i++;
      }

      if (i >= m_outputs.size()) {
        m_finished = true; // ! kind of a hack
        return;
      }
      m_last_index = i;
    }

    TimedOutput out = m_outputs.get(m_last_index);

    this.m_swerve.driveWithInput(new Translation2d(out.leftX,   out.leftY),
                                 new Translation2d(-out.rightX, out.rightY),
                                 true);
    System.out.println("PLAYING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_input.close();
    m_swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
