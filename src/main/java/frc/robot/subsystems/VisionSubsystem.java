// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera leftCamera;
  PhotonCamera rightCamera;
  PhotonPoseEstimator gerryPhotonEstimator;
  PhotonPoseEstimator geofferyPhotonEstimator;

  PhotonCameraSim gerrySim;
  PhotonCameraSim geofferySim;
  VisionSystemSim visionSim;

  AprilTagFieldLayout tags;

  public VisionSubsystem() {
    leftCamera = new PhotonCamera("gerry");
    rightCamera = new PhotonCamera("geoffery");
    tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    gerryPhotonEstimator = new PhotonPoseEstimator(tags, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(
      new Translation3d(
        Units.inchesToMeters(9.875),
        Units.inchesToMeters(11.5),
        Units.inchesToMeters(8.32874)),
      new Rotation3d(
        0,
        Units.degreesToRadians(-28.125),
        Units.degreesToRadians(30))
    ));
    geofferyPhotonEstimator = new PhotonPoseEstimator(tags, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(
      new Translation3d(
        Units.inchesToMeters(9.875),
        Units.inchesToMeters(-11.5),
        Units.inchesToMeters(8.32874)),
      new Rotation3d(
        0,
        Units.degreesToRadians(-28.125),
        Units.degreesToRadians(-30))
    ));

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(tags);
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(75));
      cameraProp.setCalibError(0.7, 0.2);
      cameraProp.setFPS(50);
      cameraProp.setAvgLatencyMs(33);
      cameraProp.setLatencyStdDevMs(15);

      gerrySim = new PhotonCameraSim(leftCamera, cameraProp);
      geofferySim = new PhotonCameraSim(rightCamera, cameraProp);
      visionSim.addCamera(gerrySim, new Transform3d(
        new Translation3d(
          Units.inchesToMeters(9.875),
          Units.inchesToMeters(11.5),
          Units.inchesToMeters(8.32874)),
        new Rotation3d(
          0,
          Units.degreesToRadians(-28.125),
          Units.degreesToRadians(30))
      ));
      visionSim.addCamera(geofferySim, new Transform3d(
        new Translation3d(
          Units.inchesToMeters(9.875),
          Units.inchesToMeters(-11.5),
          Units.inchesToMeters(8.32874)),
        new Rotation3d(
          0,
          Units.degreesToRadians(-28.125),
          Units.degreesToRadians(-30))
      ));
      gerrySim.enableDrawWireframe(true);
      geofferySim.enableDrawWireframe(true);
    }
  }
  
  public Optional<EstimatedRobotPose> getEstimatedGerryPose() {
    var result = leftCamera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    if (Robot.isSimulation()) {
      gerryPhotonEstimator.update(result).ifPresentOrElse(
        est ->
          getSimDebugField()
            .getObject("VisionEstimation")
            .setPose(est.estimatedPose.toPose2d()),
        () -> {
          getSimDebugField().getObject("VisionEstimation").setPoses();
        });
    }

    return gerryPhotonEstimator.update(result);
  }

  public Optional<EstimatedRobotPose> getEstimatedGeofferyPose() {
    var result = rightCamera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    if (Robot.isSimulation()) {
      geofferyPhotonEstimator.update(result).ifPresentOrElse(
        est ->
          getSimDebugField()
            .getObject("VisionEstimation")
            .setPose(est.estimatedPose.toPose2d()),
        () -> {
          getSimDebugField().getObject("VisionEstimation").setPoses();
        });
    }

    return geofferyPhotonEstimator.update(result);
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
