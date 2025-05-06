// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveManipulatorToPoint extends Command {
  ManipulatorSubsystem manipulator;
  double desiredAngle;

  /** Creates a new MoveManipulatorToAngle. */
  public MoveManipulatorToPoint(ManipulatorSubsystem manipulator, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.manipulator = manipulator;
    this.desiredAngle = angle;
    addRequirements(this.manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the target position once and let Motion Magic handle the profiling
    manipulator.setPosition(desiredAngle);
    SmartDashboard.putBoolean("isManipulatorMoving", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Motion Magic is running on the motor controller
    // No need to calculate and update position every cycle
    manipulator.setPosition(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("isManipulatorMoving", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(manipulator.getPosition() - (desiredAngle)) < 1.2;
  }
}
