// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  CoralSubsystem coral;
  double startTime = 0;
  double timeDiff = 0;
  /** Creates a new IntakeCoral. */
  public IntakeCoral(CoralSubsystem coral) {
    this.coral = coral;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = 0;
    timeDiff = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.intakeCoral();
    if (coral.getCurrent() > 70) {
      if (startTime == 0) {
        startTime = System.currentTimeMillis();
      } else {
        timeDiff = System.currentTimeMillis() - startTime;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeDiff > 1000;
  }
}
