// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  private VisionSubsystem vision;
  private SwerveSubsystem swerve;
  private double offset;
  private Optional<EstimatedRobotPose> estimatedPose;
  private PhotonTrackedTarget closestTarget;
  private Command pathCommand;
  private boolean isPathComplete = false;
  private Rotation2d initialHeading;

  /** Creates a new AlignToReef. */
  public AlignToReef(VisionSubsystem vision, SwerveSubsystem swerve, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.swerve = swerve;
    this.offset = offset;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("autoAlign", true);
    vision.setEnabled(true);
    initialHeading = swerve.getPose().getRotation();
    
    estimatedPose = vision.getEstimatedGeofferyPose();
    //SmartDashboard.putString("estPose", estimatedPose.toString());
    
    if (Robot.isSimulation()) {
      pathCommand = swerve.goToReef(6, 1);
      if (pathCommand != null) {
        pathCommand.schedule();
        isPathComplete = false;
      }
    }
    Logger.recordOutput("isPresent", estimatedPose.isPresent());
    if (estimatedPose.isPresent()) {
      Logger.recordOutput("estimatedPose", estimatedPose.get().estimatedPose.toPose2d());
      double closestDistance = Double.MAX_VALUE;
      closestTarget = null;
      for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
        double distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        if (distance < closestDistance) {
          closestDistance = distance;
          closestTarget = target;
        }
      }
    }
    
    if (closestTarget != null) {
      pathCommand = swerve.driveToPose(closestTarget.getFiducialId(), offset);
      SmartDashboard.putString("GoToReef", closestTarget.getFiducialId() + " " + offset);
      if (pathCommand != null) {
        pathCommand.schedule();
        isPathComplete = false;
      }
    }
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pathCommand != null && !isPathComplete) {
      isPathComplete = pathCommand.isFinished();
    } else if (pathCommand == null) {
      // Keep trying to find a tag if we didn't find one in initialize
      estimatedPose = vision.getEstimatedGeofferyPose();
      if (estimatedPose.isPresent()) {
        double closestDistance = Double.MAX_VALUE;
        closestTarget = null;
        for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
          double distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          if (distance < closestDistance) {
            closestDistance = distance;
            closestTarget = target;
          }
        }
        
        if (closestTarget != null) {
          SmartDashboard.putString("GoToReef", closestTarget.getFiducialId() + " " + offset);
          pathCommand = swerve.driveToPose(closestTarget.getFiducialId(), offset);
          if (pathCommand != null) {
            pathCommand.schedule();
            isPathComplete = false;
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("autoAlign", false);
    vision.setEnabled(false);
    if (pathCommand != null) {
      pathCommand.cancel();
    }
    
    /*if (isPathComplete && !interrupted) {
      Pose2d currentPose = swerve.getPose();
      Pose2d desiredHeading = swerve.getDesiredHeading();
      if (desiredHeading == null) {
        desiredHeading = currentPose;
      }
      Pose2d newPose = new Pose2d(currentPose.getX(), currentPose.getY(), desiredHeading.getRotation());
      
      swerve.resetOdometry(newPose);
    }*/
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isPathComplete;
  }
}
