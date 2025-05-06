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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private boolean isEnabled = false;

  public VisionSubsystem() {
    //leftCamera = new PhotonCamera("gerry");
    rightCamera = new PhotonCamera("geoffery");
    tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    /*gerryPhotonEstimator = new PhotonPoseEstimator(tags, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(
      new Translation3d(
        Units.inchesToMeters(9.875),
        Units.inchesToMeters(11.5),
        Units.inchesToMeters(8.32874)),
      new Rotation3d(
        0,
        Units.degreesToRadians(-28.125),
        Units.degreesToRadians(30))
    ));*/
    geofferyPhotonEstimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
      new Translation3d(
        Units.inchesToMeters(11.961),
        Units.inchesToMeters(-12.115),
        Units.inchesToMeters(9.112)),
      new Rotation3d(
        0,
        Units.degreesToRadians(-28.125),
        Units.degreesToRadians(49))
    ));

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(tags);
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(100));
      cameraProp.setCalibError(0.7, 0.2);
      cameraProp.setFPS(50);
      cameraProp.setAvgLatencyMs(33);
      cameraProp.setLatencyStdDevMs(15);

      //gerrySim = new PhotonCameraSim(leftCamera, cameraProp);
      geofferySim = new PhotonCameraSim(rightCamera, cameraProp);
      /*visionSim.addCamera(gerrySim, new Transform3d(
        new Translation3d(
          Units.inchesToMeters(9.875),
          Units.inchesToMeters(11.5),
          Units.inchesToMeters(8.32874)),
        new Rotation3d(
          0,
          Units.degreesToRadians(-28.125),
          Units.degreesToRadians(30))
      ));*/
      visionSim.addCamera(geofferySim, new Transform3d(
        new Translation3d(
          Units.inchesToMeters(11.961),
          Units.inchesToMeters(-12.115),
          Units.inchesToMeters(9.112)),
        new Rotation3d(
          0,
          Units.degreesToRadians(-28.125),
          Units.degreesToRadians(49))
      ));
      geofferySim.enableRawStream(true);
      geofferySim.enableProcessedStream(true);
      //gerrySim.enableDrawWireframe(true);
      geofferySim.enableDrawWireframe(true);
    }
  }

  public void updateVisionSim(Pose2d pose) {
    visionSim.update(pose);
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public void setEnabled(boolean enabled) {
    isEnabled = enabled;
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

  public Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose poseEstimate) {
    int numTags = poseEstimate.targetsUsed.size();
    if (numTags == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double avgDistance = 0.0;
    double avgAmbiguity = 0.0;
    double minDistance = Double.MAX_VALUE;
    boolean hasLowAmbiguityTag = false;

    // Calculate metrics from all visible tags
    for (PhotonTrackedTarget target : poseEstimate.targetsUsed) {
      double distance = target.getBestCameraToTarget().getTranslation().toTranslation2d()
          .getDistance(poseEstimate.estimatedPose.toPose2d().getTranslation());
      
      avgDistance += distance;
      avgAmbiguity += target.getPoseAmbiguity();
      minDistance = Math.min(minDistance, distance);
      
      // Check if we have at least one high-confidence tag
      if (target.getPoseAmbiguity() < 0.2) {
        hasLowAmbiguityTag = true;
      }
    }
    
    avgDistance /= numTags;
    avgAmbiguity /= numTags;
    
    // Base standard deviations depend on tag count and quality
    // Start with reasonable defaults
    double xyStdDev;
    double rotStdDev;
    
    if (numTags >= 3) {
      // Multiple tags provide excellent pose estimation
      xyStdDev = 0.3;
      rotStdDev = 0.4;
    } else if (numTags == 2) {
      // Two tags are reasonably good
      xyStdDev = 0.5;
      rotStdDev = 0.8;
    } else {
      // Single tag is less reliable
      xyStdDev = 1.0;
      rotStdDev = 2.0;
      
      // If the single tag has high ambiguity, trust it even less
      if (avgAmbiguity > 0.4 && !hasLowAmbiguityTag) {
        xyStdDev *= 1.5;
        rotStdDev *= 2.0;
      }
    }
    
    // Distance-based scaling - more gradual than original
    // Apply quadratic scaling but with more reasonable growth
    double distanceFactor = 1.0 + (avgDistance * avgDistance / 40.0);
    
    // Hard cap for extreme distances with single tag
    if (numTags == 1 && avgDistance > 6.0) {
      distanceFactor *= 3.0;
    }
    
    // Apply confidence multiplier based on ambiguity
    double ambiguityFactor = 1.0 + (avgAmbiguity * 2.0);
    
    // Calculate final stdDevs
    xyStdDev *= distanceFactor * ambiguityFactor;
    rotStdDev *= distanceFactor * ambiguityFactor;
    
    // Cap maximum values to prevent total rejection of vision
    xyStdDev = Math.min(xyStdDev, 10.0);
    rotStdDev = Math.min(rotStdDev, 15.0);
    
    // Only completely reject in extreme cases
    if ((numTags == 1 && avgAmbiguity > 0.8 && avgDistance > 7.0) || avgAmbiguity > 0.95) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    
    // Log values for debugging
    SmartDashboard.putNumber("Vision/AvgDistance", avgDistance);
    SmartDashboard.putNumber("Vision/AvgAmbiguity", avgAmbiguity);
    SmartDashboard.putNumber("Vision/NumTags", numTags);
    SmartDashboard.putNumber("Vision/XYStdDev", xyStdDev);
    SmartDashboard.putNumber("Vision/RotStdDev", rotStdDev);
    
    return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
