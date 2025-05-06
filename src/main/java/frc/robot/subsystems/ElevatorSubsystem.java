package frc.robot.subsystems;

import java.util.HashMap;
import java.util.ArrayList;
import frc.robot.CoralHeights;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // CAN IDs for the Neo Vortex motors
    private static final int LEFT_MOTOR_CAN_ID = 15;
    private static final int RIGHT_MOTOR_CAN_ID = 16;
    
    // PID constants
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;
    private static final double maxVel = 1200;
    private static final double maxAccel = 600;
    private static final double allowedErr = 0.1;

    private static HashMap<Integer, CoralHeights> blueCoralHeights = new HashMap<>();
    private static HashMap<Integer, CoralHeights> redCoralHeights = new HashMap<>();
    
    // Motor controllers
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;

    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig rightConfig;
    
    // Encoders
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    
    // PID controllers
    private final SparkClosedLoopController leftPIDController;
    private final SparkClosedLoopController rightPIDController;
    
    // Current setpoint
    private double currentSetpoint = 0.0;
    
    public ElevatorSubsystem() {
        // Initialize the coral heights
        blueCoralHeights.put(17, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        blueCoralHeights.put(18, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        blueCoralHeights.put(19, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        blueCoralHeights.put(20, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        blueCoralHeights.put(21, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        blueCoralHeights.put(22, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));

        redCoralHeights.put(6, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        redCoralHeights.put(7, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        redCoralHeights.put(8, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        redCoralHeights.put(9, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        redCoralHeights.put(10, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));
        redCoralHeights.put(11, new CoralHeights(new double[] {23, 35, 50.5, 78}, new double[] {23, 35, 50.5, 78}));


        // Initialize motor controllers - Neo Vortex is a brushless motor
        leftMotor = new SparkFlex(LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkFlex(RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        
        // Configure motors
        // NOTE: One motor will need to be inverted as they are mounted opposite each other
        leftConfig = new SparkFlexConfig();
        rightConfig = new SparkFlexConfig();

        leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80).follow(leftMotor, true);

        leftConfig.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-0.3, 0.3);

        rightConfig.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-0.3, 0.3);

        leftConfig.closedLoop.maxMotion
        .maxVelocity(maxVel)
        .maxAcceleration(maxAccel)
        .allowedClosedLoopError(allowedErr);

        rightConfig.closedLoop.maxMotion
        .maxVelocity(maxVel)
        .maxAcceleration(maxAccel)
        .allowedClosedLoopError(allowedErr);
        
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        
        // Get PID controllers
        leftPIDController = leftMotor.getClosedLoopController();
        rightPIDController = rightMotor.getClosedLoopController();
        
        // Reset encoder positions
        resetEncoders();
    }
    
    /**
     * Configure the PID controller with standard values
     */

    public double smoothElevatorPower(double startPos, double currentPos, double desiredPos, double maxPower) {
        // Calculate normalized position (progress from 0 to 1)
        double totalDistance = Math.abs(desiredPos - startPos);
        if (totalDistance < 0.001) return 0.1; // Avoid division by zero
        
        double normalizedPosition = Math.abs(currentPos - startPos) / totalDistance;
        
        // Implement trapezoidal profile using the provided function:
        // -2.5*|x-0.2|-2.5*|x-0.8|+2.5
        double scaleFactor = -2.5 * Math.abs(normalizedPosition - 0.2) 
                           - 2.5 * Math.abs(normalizedPosition - 0.8) 
                           + 2.5;
        
        // Clamp the value between 0 and 1
        scaleFactor = Math.max(0, Math.min(1, scaleFactor));
        
        return scaleFactor * maxPower;
    }
    
    /**
     * Reset encoder positions to zero
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    /**
     * Get the current position of the elevator
     */
    public double getPosition() {
        return leftEncoder.getPosition();
    }
    
    /**
     * Set the target position for the elevator
     * @param position The target position in encoder units
     */
    /*public void setPosition(double position) {
        currentSetpoint = position;
        SmartDashboard.putNumber("PID position", position);
        leftPIDController.setReference(position, ControlType.kPosition);
        rightPIDController.setReference(-1 * position, ControlType.kPosition);
    }*/

    public void setPosition(double position, double kFF) {
        currentSetpoint = position;
        SmartDashboard.putNumber("PID position", position);
        leftPIDController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, kFF);
        rightPIDController.setReference(-1 * position, ControlType.kPosition, ClosedLoopSlot.kSlot0, kFF);
    }
    
    /**
     * Manual control of the elevator (for testing or manual control)
     * @param speed Speed value from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        leftMotor.set(speed);
    }
    
    /**
     * Stop the elevator
     */
    public void stop() {
        leftMotor.set(0);
    }

    public double getLeftMotorCurrent() {
        return leftMotor.getOutputCurrent();
    }

    public double getRightMotorCurrent() {
        return rightMotor.getOutputCurrent();
    }
    
    /**
     * Homes the elevator by moving it down slowly until current spike is detected
     * @param currentThreshold The current threshold in amps that indicates the elevator has hit the bottom
     * @return true if homing is complete, false if still in progress
     */
    public boolean homeElevator(double currentThreshold) {
        final double HOMING_SPEED = -0.1; // Slow downward speed
        
        // Read the current draw from the motors
        double leftCurrent = leftMotor.getOutputCurrent();
        double rightCurrent = rightMotor.getOutputCurrent();
        double averageCurrent = (leftCurrent + rightCurrent) / 2.0;
        
        // If current exceeds threshold, we've hit the bottom
        if (averageCurrent > currentThreshold) {
            // Stop the elevator
            stop();
            // Reset encoder positions to zero
            resetEncoders();
            // Homing is complete
            return true;
        } else {
            // Continue moving down slowly
            setSpeed(HOMING_SPEED);
            // Homing is still in progress
            return false;
        }
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Publish current positions to SmartDashboard for debugging
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Setpoint", currentSetpoint);
        SmartDashboard.putNumber("Elevator Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Right Current", rightMotor.getOutputCurrent());
    }

    /**
     * Get the blue coral heights map
     * @return HashMap mapping AprilTag IDs to CoralHeights objects for blue alliance
     */
    public HashMap<Integer, CoralHeights> getBlueCoralHeights() {
        return blueCoralHeights;
    }
    
    /**
     * Get the red coral heights map
     * @return HashMap mapping AprilTag IDs to CoralHeights objects for red alliance
     */
    public HashMap<Integer, CoralHeights> getRedCoralHeights() {
        return redCoralHeights;
    }
}
