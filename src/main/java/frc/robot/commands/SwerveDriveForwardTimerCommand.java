// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveForwardTimerCommand extends Command {
  /** Creates a new SwerveDriveForwardTimer. */
  private final SwerveSubsystem m_swerve;

  private final double speed, time;
  private final int direction;

  private double timeToStop;
  private double[] speedArray = {0., 0., 0.};

  public SwerveDriveForwardTimerCommand(SwerveSubsystem swerve, double speed, int direction, double time) {
    this.m_swerve = swerve;
    this.speed = speed;
    this.direction = direction;
    this.time = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(getName() + " started!");

    System.out.print("command duration: ");
    System.out.println(time);

    // actual time the timer will end
    timeToStop = Timer.getFPGATimestamp() + time;
    SmartDashboard.putNumber("timeToStop", timeToStop);
    
    System.out.print("timeToStop: ");
    System.out.println(timeToStop);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // robot is moved until timer expires via isTimedOut() method
    // check which direction
    setDirectionalSpeed(speed, direction);
     
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speedArray[0], speedArray[1], speedArray[2]);
    m_swerve.setChasisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(getName() + " ended!");
    SmartDashboard.putNumber("currentTime", Timer.getFPGATimestamp());
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0., 0., 0.);
    m_swerve.setChasisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isTimedOut();
  }

  private boolean isTimedOut(){
    return Timer.getFPGATimestamp() >= timeToStop;
  }

  private void setDirectionalSpeed(double speed, int direction){
    for ( int i =0; i < speedArray.length; i++) {
      speedArray[i] = 0.;
    }

    switch ( direction ) {
      case 0:
        speedArray[0] = speed;
        break;
      case 1:
        speedArray[1] = speed;
        break;
      case 2:
        speedArray[2] = speed;
        break;
      default:
        break;
    }
  }
}
