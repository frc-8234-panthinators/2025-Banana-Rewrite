// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectAlgaeCurrent extends Command {
  AlgaeSubsystem algae;
  double startTime;
  double timeDiff;
  double currentTime;
  /** Creates a new DetectAlgaeCurrent. */
  public DetectAlgaeCurrent(AlgaeSubsystem algae) {
    this.algae = algae;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    currentTime = System.currentTimeMillis();
    timeDiff = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((currentTime - startTime) > 500) {
      var current = algae.getCurrent();
      if (current > 38) {
        System.out.println("Current spike detected at: " + current);
        timeDiff = System.currentTimeMillis() - currentTime;
      }
    } else {
      currentTime = System.currentTimeMillis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeDiff > 1000;
  }
}
