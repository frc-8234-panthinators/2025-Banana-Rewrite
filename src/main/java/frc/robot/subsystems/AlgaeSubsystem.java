package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax algaeMotor;
    private final SparkMaxConfig algaeConfig;

    public AlgaeSubsystem() {
        algaeMotor = new SparkMax(40, MotorType.kBrushless); //TODO: Change to correct port
        algaeConfig = new SparkMaxConfig();

        algaeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intakeAlgae() {
        algaeMotor.set(0.4);
    }

    public double getCurrent() {
        var current = algaeMotor.getOutputCurrent();
        SmartDashboard.putNumber("Algae Current", current);
        return current;
    }

    public void outtakeAlgae() {
        algaeMotor.set(-1);
    }

    public void stopAlgae() {
        algaeMotor.set(0);
    }
}
