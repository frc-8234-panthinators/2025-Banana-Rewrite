package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;

public class CoralSubsystem extends SubsystemBase {
    private final SparkFlex coralMotor;

    public CoralSubsystem() {
        coralMotor = new SparkFlex(41, MotorType.kBrushless); //TODO: Change to correct port
    }

    public void intakeCoral() {
        coralMotor.set(0.4);
    }

    public double getCurrent() {
        return coralMotor.getOutputCurrent();
    }

    public void outtakeCoral() {
        coralMotor.set(-0.4);
    }

    public void stopCoral() {
        coralMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("coralCurrent", getCurrent());
    }
}
