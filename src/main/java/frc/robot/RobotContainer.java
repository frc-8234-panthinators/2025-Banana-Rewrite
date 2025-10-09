// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.Autos;
import frc.robot.commands.DetectAlgaeCurrent;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveElevatorToPoint;
import frc.robot.commands.MoveElevatorToPointIn;
import frc.robot.commands.MoveManipulatorToAngle;
import frc.robot.commands.MoveManipulatorToPoint;
import frc.robot.commands.OuttakeAlgae;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveElevatorToLeftL1;
import frc.robot.commands.MoveElevatorToLeftL2;
import frc.robot.commands.MoveElevatorToLeftL3;
import frc.robot.commands.MoveElevatorToLeftL4;
import frc.robot.commands.OuttakeCoral;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  SwerveSubsystem swerve;
  XBoxContainer xbox;
  ElevatorSubsystem elevator;
  ManipulatorSubsystem manipulator;
  VisionSubsystem vision;
  AlgaeSubsystem algae;
  CoralSubsystem coral;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SwerveSubsystem swerve, XBoxContainer xbox, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator, VisionSubsystem vision, AlgaeSubsystem algae, CoralSubsystem coral) {
    this.swerve = swerve;
    this.xbox = xbox;
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.vision = vision;
    this.algae = algae;
    this.coral = coral;

    NamedCommands.registerCommand("AutoAlignLeft", new AlignToReef(vision, swerve, 0.086));
    NamedCommands.registerCommand("MoveToL4", new MoveElevatorToPointIn(elevator, -78).andThen(new MoveManipulatorToAngle(manipulator, 66)));
    NamedCommands.registerCommand("OuttakeCoral", new OuttakeCoral(coral));
    NamedCommands.registerCommand("MoveToBottomAlgae", getAutonomousCommand()); //TODO: Rotate manipulator
    NamedCommands.registerCommand("IntakeAlgae", new IntakeAlgae(algae));
    NamedCommands.registerCommand("MoveToBarge", new MoveElevatorToPoint(elevator, -55.2)); //TODO: Rotate manipulator
    NamedCommands.registerCommand("OuttakeAlgae", new OuttakeAlgae(algae));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //xbox.alignLeft.whileTrue(new AlignToReef(vision, swerve, 0.086));
    //xbox.alignRight.whileTrue(new AlignToReef(vision, swerve, -0.26));
    xbox.dpadDownL1.onTrue(new MoveElevatorToPointIn(elevator, -23).andThen(new MoveManipulatorToAngle(manipulator,70)));
    xbox.dpadLeftL2.onTrue(new MoveElevatorToPointIn(elevator, -34).andThen(new MoveManipulatorToAngle(manipulator, 66)));
    xbox.dpadRightL3.onTrue(new MoveElevatorToPointIn(elevator, -44).andThen(new MoveManipulatorToAngle(manipulator, 66)));
    xbox.dpadUpL4.onTrue(new MoveElevatorToPoint(elevator, -55.2).andThen(new MoveManipulatorToAngle(manipulator, 66)));

    //xbox.dpadDownL1.onTrue(new MoveManipulatorToPoint(manipulator, 0).andThen(new MoveElevatorToPoint(elevator, -2).andThen(new HomeElevator(elevator))));
    
    xbox.cancel.onTrue(new MoveManipulatorToPoint(manipulator, 0).andThen(new MoveElevatorToPoint(elevator, -6).andThen(new HomeElevator(elevator))));
    xbox.coralIntake.onTrue(new MoveManipulatorToPoint(manipulator, -16).andThen(new IntakeCoral(coral)));
    xbox.coralOuttake.onTrue(new OuttakeCoral(coral));

    xbox.algaeIntake.onTrue(new MoveElevatorToPointIn(elevator, -22).andThen(new IntakeAlgae(algae)).andThen(new MoveManipulatorToPoint(manipulator, -5.5)).andThen(new DetectAlgaeCurrent(algae)).andThen(new MoveManipulatorToPoint(manipulator, 0)).andThen(new MoveElevatorToPoint(elevator, -2).andThen(new HomeElevator(elevator))));
    xbox.upperAlgae.onTrue(new MoveElevatorToPointIn(elevator, -34).andThen(new IntakeAlgae(algae)).andThen(new MoveManipulatorToPoint(manipulator, -5.5)).andThen(new DetectAlgaeCurrent(algae)).andThen(new MoveManipulatorToPoint(manipulator, 0)).andThen(new MoveElevatorToPoint(elevator, -2).andThen(new HomeElevator(elevator))));
    xbox.algaeOuttake.onTrue(new OuttakeAlgae(algae));
    xbox.start.onTrue(new MoveManipulatorToPoint(manipulator, -14));
    xbox.back.onTrue(new MoveElevatorToPoint(elevator, -55.2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new PathPlannerAuto("Back out 4");
    return new Command() {
      
    };
  }
}
