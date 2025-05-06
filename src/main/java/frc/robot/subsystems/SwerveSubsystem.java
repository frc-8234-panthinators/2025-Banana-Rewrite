package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    public SwerveSubsystem() {
        try {
            double maximumSpeed = Units.feetToMeters(20);
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
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
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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

    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, true, false);
        //swerveDrive.driveFieldOriented(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }

    public void updateOdometry() {
        swerveDrive.updateOdometry();
        //SmartDashboard.putNumber("rotation", swerveDrive.getOdometryHeading().getDegrees());
    }

    public void resetHeading() {
        swerveDrive.zeroGyro();
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