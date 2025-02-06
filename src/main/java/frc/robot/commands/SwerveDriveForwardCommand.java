// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.TextUI;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveForwardCommand extends Command {
  /** Creates a new SwerveDriveForward. */
  private final SwerveSubsystem m_swerve;

  private final double forwardSpeed, leftSpeed, turnSpeed;


  public SwerveDriveForwardCommand(SwerveSubsystem swerve, double forwardSpeed,
                                   double leftSpeed, double turnSpeed) {

    this.m_swerve = swerve;
    this.forwardSpeed = forwardSpeed;
    this.leftSpeed = leftSpeed;
    this.turnSpeed = turnSpeed;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(getName() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed);
    m_swerve.showChasisSpeedsOnLogger("execute");
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
