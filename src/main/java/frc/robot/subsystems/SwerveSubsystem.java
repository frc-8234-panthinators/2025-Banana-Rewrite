package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    private HashMap<Integer,Pose2d> redReefPositions = new HashMap<>();
    private HashMap<Integer,Pose2d> blueReefPositions = new HashMap<>();
    private Pose2d desiredHeading;
    public SwerveSubsystem() {
        redReefPositions.put(6, new Pose2d(13.72, 2.873, Rotation2d.fromDegrees(300-180)));
        redReefPositions.put(7, new Pose2d(14.39, 4.026, Rotation2d.fromDegrees(0-180)));
        redReefPositions.put(8, new Pose2d(13.72, 5.179, Rotation2d.fromDegrees(60-180)));
        redReefPositions.put(9, new Pose2d(12.39, 5.179, Rotation2d.fromDegrees(120-180)));
        redReefPositions.put(10, new Pose2d(11.73, 4.026, Rotation2d.fromDegrees(180-180)));
        redReefPositions.put(11, new Pose2d(12.39, 2.873, Rotation2d.fromDegrees(240-180)));
        
        blueReefPositions.put(17, new Pose2d(3.824, 2.873, Rotation2d.fromDegrees(240-180)));
        blueReefPositions.put(18, new Pose2d(3.158, 4.026, Rotation2d.fromDegrees(0)));
        blueReefPositions.put(19, new Pose2d(3.824, 5.179, Rotation2d.fromDegrees(120-180)));
        blueReefPositions.put(20, new Pose2d(5.155, 5.179, Rotation2d.fromDegrees(60-180)));
        blueReefPositions.put(21, new Pose2d(5.821, 4.026, Rotation2d.fromDegrees(0-180)));
        blueReefPositions.put(22, new Pose2d(5.155, 2.873, Rotation2d.fromDegrees(300-180)));
        
        try {
            double maximumSpeed = Units.feetToMeters(20);
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
            //swerveDrive.setGyroOffset(new Rotation3d(0, 0, (3*Math.PI)/2));
        } catch(IOException err) {
            err.printStackTrace();
        }

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants - reduced from 5.0
                        new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants - reduced from 5.0
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    public Command driveToPose(int reefNumber, double offset) {
        return defer(() -> goToReef(reefNumber, offset));
    }

    public Command goToReef(int reefNumber, double offset) {
        Logger.recordOutput("reefNumber", reefNumber);
        Pose2d targetPose = redReefPositions.get(reefNumber);
        if (targetPose == null) {
            targetPose = blueReefPositions.get(reefNumber);
        }
        if (targetPose == null) {
            Logger.recordOutput("foundReef", false);
            return null;
        }
        Logger.recordOutput("foundReef", true);
        // Apply the offset to the target pose
        // This is a simplistic approach - you might want to offset perpendicular to the target angle
        Pose2d offsetPose = new Pose2d(
            targetPose.getX() + offset * Math.cos(targetPose.getRotation().getRadians() + Math.PI/2),
            targetPose.getY() + offset * Math.sin(targetPose.getRotation().getRadians() + Math.PI/2),
            targetPose.getRotation()
        );
        

        Logger.recordOutput("desiredPose", offsetPose);
        // Adjusted path constraints for smoother motion
        PathConstraints constraints = new PathConstraints(2.0, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540));
        return AutoBuilder.pathfindToPose(offsetPose, constraints, 0.0);
    }

    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation,true, false);
        //swerveDrive.driveFieldOriented(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }

    public void updateOdometry() {
        swerveDrive.updateOdometry();
        //SmartDashboard.putNumber("rotation", swerveDrive.getOdometryHeading().getDegrees());
    }

    public void resetHeading() {
        swerveDrive.zeroGyro();
        desiredHeading = swerveDrive.getPose();
        //swerveDrive.setGyroOffset(new Rotation3d(0, 0, (3*Math.PI)/2));
    }

    public Rotation2d getHeading() {
        return swerveDrive.getGyro().getRotation3d().toRotation2d();
    }

    public Pose2d getDesiredHeading() {
        return desiredHeading;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> matrix) {
        swerveDrive.addVisionMeasurement(pose, timestamp, matrix);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        swerveDrive.addVisionMeasurement(pose, timestamp);
    }
    
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRelative(ChassisSpeeds chassis) {
        swerveDrive.drive(chassis);
    }
}