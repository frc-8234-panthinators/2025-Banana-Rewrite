// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command that aligns the robot to an AprilTag on the reef and then
 * positions the elevator to the L1 position (first position in the CoralHeights array)
 * on the left side, based on which AprilTag it aligned to.
 */
public class AlignAndPositionElevator {
  
  /**
   * Creates a new command that first aligns to a reef AprilTag and then moves the elevator
   * to the L1 position based on which AprilTag was detected.
   *
   * @param vision The vision subsystem
   * @param swerve The swerve subsystem 
   * @param elevator The elevator subsystem
   * @param offset The offset for the AlignToReef command
   * @return The command sequence
   */
  public static Command create(VisionSubsystem vision, SwerveSubsystem swerve, 
                               ElevatorSubsystem elevator, double offset) {
    return Commands.sequence(
        new AlignToReef(vision, swerve, offset),
        new MoveElevatorToLeftL1(vision, elevator)
    );
  }
  
  // Private constructor to prevent instantiation
  private AlignAndPositionElevator() {}
} 