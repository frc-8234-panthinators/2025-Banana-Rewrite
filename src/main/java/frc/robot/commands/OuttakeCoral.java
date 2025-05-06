// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCoral extends Command {
  CoralSubsystem coral;
  double startTime;
  /** Creates a new OuttakeCoral. */
  public OuttakeCoral(CoralSubsystem coral) {
    this.coral = coral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.outtakeCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > 1000;
  }
}
