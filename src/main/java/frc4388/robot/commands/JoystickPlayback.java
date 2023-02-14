// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
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
    public double leftX  = 0.0;
    public double leftY  = 0.0;
    public double rightX = 0.0;
    public double rightY = 0.0;

    public long timedOffset = 0;
  }

  private final SwerveDrive            swerve;
  private       Scanner                input;
  private final ArrayList<TimedOutput> outputs;
  private       int                    counter      = 0;
  private       long                   startTime    = 0;
  private       long                   playbackTime = 0;
  private       int                    lastIndex;
  private       boolean                m_finished   = false; // ! find a better way

  /** Creates a new JoystickPlayback. */
  public JoystickPlayback(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    outputs = new ArrayList<>();

    addRequirements(this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    playbackTime = 0;
    lastIndex    = 0;
    try {
      input = new Scanner(new File("/home/lvuser/JoystickInputs.txt"));

      String line = "";
      while (input.hasNextLine()) {
        line = input.nextLine();
      
        String[] values = line.split(",");

        var out = new TimedOutput();
        out.leftX  = Double.parseDouble(values[0]);
        out.leftY  = Double.parseDouble(values[1]);
        out.rightX = Double.parseDouble(values[2]);
        out.rightY = Double.parseDouble(values[3]);

        out.timedOffset = Long.parseLong(values[4]);

        outputs.add(out);
      }

      input.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    System.out.println("STARTING PLAYBACK");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter == 0) {
      startTime = System.currentTimeMillis();
      playbackTime = 0;
    } else {
      playbackTime = System.currentTimeMillis() - startTime;
    }

    // skip to reasonable time frame
    // too tired to write comment: ask daniel thomas; it goes to the thing until it's bigger than the other thing
    {
      int i = lastIndex == 0 ? 1 : lastIndex;
      while (i < outputs.size() && outputs.get(i).timedOffset < playbackTime) {
        i++;
      }

      if (i >= outputs.size()) {
        m_finished = true; // ! kind of a hack
        return;
      }
      lastIndex = i;
    }

    TimedOutput lastOut = outputs.get(lastIndex - 1);
    TimedOutput out     = outputs.get(lastIndex);

    double deltaTime     = out.timedOffset - lastOut.timedOffset;
    double playbackDelta = playbackTime    - lastOut.timedOffset;

    double lerpLX = lastOut.leftX  + (out.leftX  - lastOut.leftX)  * (playbackDelta / deltaTime);
    double lerpLY = lastOut.leftY  + (out.leftY  - lastOut.leftY)  * (playbackDelta / deltaTime);
    double lerpRX = lastOut.rightX + (out.rightX - lastOut.rightX) * (playbackDelta / deltaTime);
    double lerpRY = lastOut.rightY + (out.rightY - lastOut.rightY) * (playbackDelta / deltaTime);

    // this.swerve.driveWithInput(new Translation2d(out.leftX,   out.leftY),
    //                              new Translation2d(out.rightX, out.rightY),
    //                              true);
    
    this.swerve.driveWithInput( new Translation2d(lerpLX, lerpLY),
                                new Translation2d(lerpRX, lerpRY),
                                true);
                                
    System.out.println("PLAYING");
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    input.close();
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
