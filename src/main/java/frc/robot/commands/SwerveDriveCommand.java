// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
  /** Creates a new SwerveDriveCommand. */
  private final SwerveSubsystem m_swerve;
  // run functions and use the return value within each command
  private final Supplier<Double> m_moveForwardFunction, m_moveLeftFunction, m_turnFunction;


  public SwerveDriveCommand(SwerveSubsystem swerve, Supplier<Double> moveForward, Supplier<Double> moveLeft, Supplier<Double> rotate) {
    this.m_swerve = swerve;
    this.m_moveLeftFunction = moveForward;
    this.m_moveForwardFunction = moveLeft;
    this.m_turnFunction = rotate;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(getName() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get joystick input
    double forwardSpeed = m_moveForwardFunction.get();
    double leftSpeed = m_moveLeftFunction.get();
    double turnSpeed = m_turnFunction.get();

    // apply dead-band to avoid jitter on the joystick
    forwardSpeed = Math.abs(forwardSpeed) > OperatorConstants.kDeadband ? forwardSpeed : 0.0;
    leftSpeed = Math.abs(leftSpeed) > OperatorConstants.kDeadband ? leftSpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > OperatorConstants.kDeadband ? turnSpeed : 0.0;

    // make driving smoother

    // chasis speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed);
    m_swerve.showChasisSpeedsOnLogger("joystick");
    m_swerve.setChasisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(getName() + " ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
