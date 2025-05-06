// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeElevator extends Command {
  ElevatorSubsystem elevator;
  final double HOMING_SPEED = 0.1;
  double averageCurrent;
  /** Creates a new HomeElevator. */
  public HomeElevator(ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCurrent = elevator.getLeftMotorCurrent();
    double rightCurrent = elevator.getRightMotorCurrent();
    averageCurrent = (leftCurrent + rightCurrent) / 2.0;
    elevator.setSpeed(HOMING_SPEED);
    SmartDashboard.putBoolean("isHomed", false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    elevator.resetEncoders();
    SmartDashboard.putBoolean("isHomed", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return averageCurrent > 40;
  }
}
