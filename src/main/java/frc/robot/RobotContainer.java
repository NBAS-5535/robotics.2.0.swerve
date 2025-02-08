// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveDriveForwardCommand;
import frc.robot.commands.SwerveDriveForwardTimerCommand;
import frc.robot.subsystems.SwerveSubsystem;

import javax.management.relation.RelationException;

import com.ctre.phoenix6.Utils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //private final CommandJoystick m_joystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
    
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem m_swerve = new SwerveSubsystem(m_driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        m_swerve.setDefaultCommand(new SwerveDriveCommand(m_swerve, 
                                                      () -> -m_driverController.getRawAxis(OperatorConstants.kDriverYAxis),
                                                      () -> m_driverController.getRawAxis(OperatorConstants.kDriverXAxis),
                                                      () -> m_driverController.getRawAxis(OperatorConstants.kDriverRotAxis)
                                                      ));
        m_driverController.b().whileTrue(new SwerveDriveForwardCommand(m_swerve, 0.25, 0., 0.));
        m_driverController.a().whileTrue(new SwerveDriveForwardCommand(m_swerve, 0., 0., 0.));
    
        m_driverController.x().whileTrue(new SwerveDriveForwardTimerCommand(m_swerve, 0.6, 0, 10.));
        m_driverController.y().whileTrue(new InstantCommand(() -> m_swerve.setModuleTestSpeed()));                                              
  }    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    //Autos.autoDriveByTime(m_swerve);
    /*
    return new SequentialCommandGroup(new SwerveDriveForwardTimerCommand(m_swerve, 0.25, 0, 5.),
                                      new SwerveDriveForwardTimerCommand(m_swerve, 0.6, 1, 10.),
                                      new SwerveDriveForwardCommand(m_swerve, 0., 0., 0.));
    */
    return Autos.autoDriveByTime(m_swerve);
  }

  public double setToZero() {
    return 0.0;
  }

  
}
