// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.Swerve;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.UtilityStructs.TimedOutput;
import frc4388.utility.controller.XboxController;
import frc4388.robot.subsystems.Arm;

public class JoystickRecorder extends CommandBase {
  public final SwerveDrive            swerve;
  public final XboxController         driveXbox;
  public final XboxController         operatorXbox;
  public final Arm                    arm;
  private       String                 filename;
  public  final ArrayList<TimedOutput> outputs = new ArrayList<>();
  private       long                   startTime = -1;

  /** Creates a new JoystickRecorder. */
  public JoystickRecorder(SwerveDrive swerve, XboxController driveXbox, 
  XboxController operatorXbox, Arm arm, String filename) {
    this.swerve = swerve;
    this.driveXbox = driveXbox;
    this.operatorXbox = operatorXbox;
    this.arm = arm;
    this.filename = filename;

    addRequirements(this.swerve);
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outputs.clear();

    this.startTime = System.currentTimeMillis();

    outputs.add(new TimedOutput());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var inputs = new TimedOutput();
    inputs.driverLeftX  = driveXbox.getLeftXAxis();
    inputs.driverLeftY  = driveXbox.getLeftYAxis();
    inputs.driverRightX = driveXbox.getRightXAxis();
    inputs.driverRightY = driveXbox.getRightYAxis();

    inputs.operatorLeftX  = driveXbox.getLeftXAxis();
    inputs.operatorLeftY  = driveXbox.getLeftYAxis();
    inputs.operatorRightX = driveXbox.getRightXAxis();
    inputs.operatorRightY = driveXbox.getRightYAxis();

    inputs.timedOffset = System.currentTimeMillis() - startTime;

    outputs.add(inputs);

    swerve.driveWithInput(new Translation2d(inputs.driverLeftX,  inputs.driverLeftX),
                          new Translation2d(inputs.driverRightX, inputs.driverRightY),
                          true);
    arm.setRotVel(inputs.operatorLeftY);
    arm.setTeleVel(inputs.operatorRightY);
    
    System.out.println("RECORDING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    File output = new File("/home/lvuser/autos/" + filename);

    try (PrintWriter writer = new PrintWriter(output)) {
      for (var input : outputs) {
        writer.println( input.driverLeftX  + "," + input.driverLeftX  + "," +
                        input.driverRightX + "," + input.driverRightY + "," +
                        input.operatorLeftX  + "," + input.operatorLeftX  + "," +
                        input.operatorRightX + "," + input.operatorRightY + "," +
                        input.timedOffset);
      }

      writer.close();
    } catch (IOException e) {
        e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
