// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPointIn extends Command {
  ElevatorSubsystem elevator;
  double desiredPos;
  double kDt = 0.02;
  TrapezoidProfile profile;
  TrapezoidProfile.State setpoint;
  TrapezoidProfile.State goal;

  /** Creates a new MoveElevatorToPoint. */
  public MoveElevatorToPointIn(ElevatorSubsystem elevator, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.desiredPos = position;
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(80, 100));
    setpoint = new TrapezoidProfile.State(elevator.getPosition(), 0d);
    goal = new TrapezoidProfile.State((1.14773*(desiredPos+7.286+12.5)), 0d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = profile.calculate(kDt, setpoint, goal);
    elevator.setPosition(setpoint.position, -0.7);
    SmartDashboard.putBoolean("isElevatorMoving", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("isElevatorMoving", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getPosition() - (1.14773*(desiredPos+7.286+12.5))) < 1;
  }
}
