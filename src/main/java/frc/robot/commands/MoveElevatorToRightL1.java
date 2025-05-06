// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.CoralHeights;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class MoveElevatorToRightL1 extends Command {
  private VisionSubsystem vision;
  private ElevatorSubsystem elevator;
  private Optional<EstimatedRobotPose> estimatedPose;
  private PhotonTrackedTarget closestTarget;
  private Command elevatorCommand;
  private int targetFiducialId = -1;
  
  /** Creates a new MoveElevatorToL1 command. */
  public MoveElevatorToRightL1(VisionSubsystem vision, ElevatorSubsystem elevator) {
    this.vision = vision;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the estimated pose from vision
    estimatedPose = vision.getEstimatedGeofferyPose();
    
    if (estimatedPose.isPresent()) {
      // Find the closest AprilTag
      double closestDistance = Double.MAX_VALUE;
      closestTarget = null;
      for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
        double distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getDistance(
            estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        if (distance < closestDistance) {
          closestDistance = distance;
          closestTarget = target;
        }
      }
      
      if (closestTarget != null) {
        targetFiducialId = closestTarget.getFiducialId();
        
        // Determine the L1 position based on AprilTag ID (blue or red alliance)
        double l1Position = 0.0;
        
        if (targetFiducialId >= 6 && targetFiducialId <= 11) {
          // Red alliance tags (6-11)
          CoralHeights heights = elevator.getRedCoralHeights().get(targetFiducialId);
          if (heights != null) {
            l1Position = heights.getRightPositions()[0]; // L1 is first position in left array
          }
        } else if (targetFiducialId >= 17 && targetFiducialId <= 22) {
          // Blue alliance tags (17-22)
          CoralHeights heights = elevator.getBlueCoralHeights().get(targetFiducialId);
          if (heights != null) {
            l1Position = heights.getRightPositions()[0]; // L1 is first position in left array
          }
        }
        
        // Create and schedule the MoveElevatorToPoint command
        elevatorCommand = new MoveElevatorToPointIn(elevator, l1Position);
        elevatorCommand.schedule();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The MoveElevatorToPoint command handles the movement
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (elevatorCommand != null) {
      elevatorCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorCommand != null && elevatorCommand.isFinished();
  }
} 