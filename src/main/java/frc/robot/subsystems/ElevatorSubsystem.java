package frc.robot.subsystems;

import java.util.HashMap;
import frc.robot.CoralHeights;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // CAN IDs for the Neo Vortex motors: update these
    private static final int LEFT_MOTOR_CAN_ID = 15;
    private static final int RIGHT_MOTOR_CAN_ID = 16;
    
    // PID constants; likely need to adjust these, especially maxVel and Accel
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.0;
    private static final double maxVel = 1200;
    private static final double maxAccel = 600;

    private static HashMap<Integer, CoralHeights> blueCoralHeights = new HashMap<>();
    private static HashMap<Integer, CoralHeights> redCoralHeights = new HashMap<>();
    
    // Motor controllers
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    
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


        // Initialize motor controllers
        leftMotor = new TalonFX(LEFT_MOTOR_CAN_ID);
        rightMotor = new TalonFX(RIGHT_MOTOR_CAN_ID);
        
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 80; // Amps: is this OK?
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;

        config.MotionMagic.MotionMagicCruiseVelocity = maxVel;
        config.MotionMagic.MotionMagicAcceleration = maxAccel;

        config.MotorOutput.PeakForwardDutyCycle = 0.3;
        config.MotorOutput.PeakReverseDutyCycle = -0.3;
        
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        rightMotor.setControl(new Follower(LEFT_MOTOR_CAN_ID, true));
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
     * Get the current position of the elevator
     */
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
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
        PositionVoltage positionRequest = new PositionVoltage(position)
        .withSlot(0)            // equivalent to kSlot0
        .withFeedForward(kFF);  // same as Spark's arbitrary feedforward

        leftMotor.setControl(positionRequest);

        // Right side moves in opposite direction
        rightMotor.setControl(positionRequest.withPosition(-position));
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

    public void resetEncoders() { //just resetting motors, idk how much I wanna change language before this works
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public void resetEncoder() {
        leftMotor.getConfigurator().setPosition(0.0);
    }

    public double getLeftMotorCurrent() {
        return leftMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getRightMotorCurrent() {
        return rightMotor.getSupplyCurrent().getValueAsDouble();
    }
    
    /**
     * Homes the elevator by moving it down slowly until current spike is detected
     * @param currentThreshold The current threshold in amps that indicates the elevator has hit the bottom
     * @return true if homing is complete, false if still in progress
     */
    public boolean homeElevator(double currentThreshold) {
        final double HOMING_SPEED = -0.1; // Slow downward speed
        
        // Read the current draw from the motors
        double leftCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
        double rightCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
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
        SmartDashboard.putNumber("Manipulator Current", leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Manipulator Current", rightMotor.getSupplyCurrent().getValueAsDouble());
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
