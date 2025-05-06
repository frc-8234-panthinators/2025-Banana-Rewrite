package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
    // CAN ID for the Kraken motor
    private static final int KRAKEN_MOTOR_CAN_ID = 30; // Adjust as needed
    
    // PID constants
    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.6; // Feedforward - voltage per rotation/second
    
    // Motion constraints
    private static final double MAX_VELOCITY = 30;
    private static final double MAX_ACCELERATION = 40;
    
    // Motor controller
    private final TalonFX krakenMotor;
    
    // Position request object for position control
    private final PositionVoltage positionRequest;
    
    // Current setpoint
    private double currentSetpoint = 0.0;
    
    /**
     * Creates a new ManipulatorSubsystem using a Kraken motor with PID position control.
     */
    public ManipulatorSubsystem() {
        // Initialize motor controller
        krakenMotor = new TalonFX(KRAKEN_MOTOR_CAN_ID);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure PID for slot 0
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        
        // Configure motion constraints
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;
        
        // Set brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 40; // Amps
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply the configuration
        krakenMotor.getConfigurator().apply(config);
        
        // Create position control request
        positionRequest = new PositionVoltage(0).withSlot(0);
        
        // Factory reset to ensure we're at a known state
        krakenMotor.getConfigurator().setPosition(0.0);
    }
    
    /**
     * Sets the target position of the manipulator.
     * 
     * @param position The target position in rotations.
     */
    public void setPosition(double position) {
        currentSetpoint = position;
        krakenMotor.setControl(positionRequest.withPosition(position));
    }
    
    /**
     * Gets the current position of the manipulator.
     * 
     * @return The current position in rotations.
     */
    public double getPosition() {
        return krakenMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Gets the current velocity of the manipulator.
     * 
     * @return The current velocity in rotations per second.
     */
    public double getVelocity() {
        return krakenMotor.getVelocity().getValueAsDouble();
    }
    
    /**
     * Manual control of the manipulator (for testing or manual control).
     * 
     * @param speed Speed value from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        krakenMotor.set(speed);
    }
    
    /**
     * Stops the manipulator.
     */
    public void stop() {
        krakenMotor.stopMotor();
    }
    
    /**
     * Resets the encoder position to zero.
     */
    public void resetEncoder() {
        krakenMotor.getConfigurator().setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        // Publish data to SmartDashboard for debugging
        SmartDashboard.putNumber("Manipulator Position", getPosition());
        SmartDashboard.putNumber("Manipulator Velocity", getVelocity());
        SmartDashboard.putNumber("Manipulator Setpoint", currentSetpoint);
        SmartDashboard.putNumber("Manipulator Current", krakenMotor.getSupplyCurrent().getValueAsDouble());
    }
}
