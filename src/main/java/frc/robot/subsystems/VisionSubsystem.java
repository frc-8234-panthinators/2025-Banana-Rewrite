// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera leftCamera = new PhotonCamera("gerry");
  PhotonCamera rightCamera = new PhotonCamera("geoffery");
  AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  PhotonPoseEstimator gerryPhotonEstimator = new PhotonPoseEstimator(tags, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(
    new Translation3d(
      Units.inchesToMeters(9.875),
      Units.inchesToMeters(11.5),
      Units.inchesToMeters(8.32874)),
    new Rotation3d(
      0,
      Units.degreesToRadians(-28.125),
      Units.degreesToRadians(30))
  ));
  PhotonPoseEstimator geofferyPhotonEstimator = new PhotonPoseEstimator(tags, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(
    new Translation3d(
      Units.inchesToMeters(9.875),
      Units.inchesToMeters(-11.5),
      Units.inchesToMeters(8.32874)),
    new Rotation3d(
      0,
      Units.degreesToRadians(-28.125),
      Units.degreesToRadians(-30))
  ));
  
  
  public Optional<EstimatedRobotPose> getEstimatedGerryPose() {
    var result = leftCamera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }
    return gerryPhotonEstimator.update(result);
  }

  public Optional<EstimatedRobotPose> getEstimatedGeofferyPose() {
    var result = rightCamera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }
    return geofferyPhotonEstimator.update(result);
  }
}
