// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoralSubsystem;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private SwerveSubsystem swerve = new SwerveSubsystem();
  private VisionSubsystem vision = new VisionSubsystem();
  private XBoxContainer xBox = new XBoxContainer();
  private ElevatorSubsystem elevator = new ElevatorSubsystem();
  private ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  private AlgaeSubsystem algae = new AlgaeSubsystem();
  private CoralSubsystem coral = new CoralSubsystem();

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.recordMetadata("RobotSim", "MapleSimTest");

    if (isSimulation()) {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    m_robotContainer = new RobotContainer(swerve, xBox, elevator, manipulator, vision, algae, coral);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    /*Optional<EstimatedRobotPose> poseGerry = vision.getEstimatedGerryPose();
    if (poseGerry.isPresent()) {
      EstimatedRobotPose poseReal = poseGerry.get();
      double lowestAmbiguity = 1;
      for (PhotonTrackedTarget target : poseReal.targetsUsed) {
        if (target.getPoseAmbiguity() < lowestAmbiguity) {
          lowestAmbiguity = target.getPoseAmbiguity();
        }
      }
      if (lowestAmbiguity < 0.2) {
        swerve.addVisionMeasurement(poseReal.estimatedPose.toPose2d(), poseReal.timestampSeconds);
      }
    }*/
    Optional<EstimatedRobotPose> poseGeoffery = vision.getEstimatedGeofferyPose();
    if (poseGeoffery.isPresent()) {
      EstimatedRobotPose poseReal = poseGeoffery.get();
      double lowestAmbiguity = 1;
    
    for (PhotonTrackedTarget target : poseReal.targetsUsed) {
      if (target.getPoseAmbiguity() < lowestAmbiguity) {
        lowestAmbiguity = target.getPoseAmbiguity();
      }
    }
    if (lowestAmbiguity <= 0.1) {
      var stdDevs = vision.calculateStdDevs(poseReal);
      swerve.addVisionMeasurement(poseReal.estimatedPose.toPose2d(), poseReal.timestampSeconds, stdDevs);
    }
}

    swerve.updateOdometry();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    vision.setEnabled(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    manipulator.setPosition(0);
    vision.setEnabled(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //manipulator.setPosition(0);
    swerve.drive(new Translation2d(xBox.driveY() * -1, xBox.driveX() * -1), xBox.rotate() * -1);
    if (xBox.getControllerXButton() == true) {
      swerve.resetHeading();
    }

    algae.getCurrent();

    /*if (Math.abs(xBox.elevatorManual()) > 0.1) {
      manipulator.setSpeed(xBox.elevatorManual() * 0.1);
      lastPos = manipulator.getPosition();
    } else {
      manipulator.setPosition(lastPos);
    }*/

    /*if (xBox.runAlgae()) {
      algae.intakeAlgae();
    } else {
      algae.outtakeAlgae();
    }*/

    /*if (xBox.runCoral()) {
      coral.intakeCoral();
    } else if (xBox.outtakeCoral()) {
      coral.outtakeCoral();
    } else {
      coral.stopCoral();
    }*/
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().resetFieldForAuto();
    //SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));
  }
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/Algae", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

    vision.updateVisionSim(swerve.getPose());

    Logger.recordOutput("RobotPose", new Pose2d());
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});
    Logger.recordOutput("FinalComponentPoses", new Pose3d[] {new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0)),
      new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0)),
      new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0)),
      new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0)),
      new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0)),
      new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0))});
  }
}
