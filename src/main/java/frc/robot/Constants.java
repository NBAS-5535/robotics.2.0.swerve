// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// swerve options: 
//  0: CTRE-generated CommandSwerveDrivetrain
//  1: Generic
public static final int swerveOptions = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 0;
    public static final int kDriverXAxis = 1;
    public static final int kDriverRotAxis = 4;

    public static final double kDeadband = 0.05;

  }

  public static final double epsilonPower = 0.05;
  public static final double epsilonController = 0.01;

  //joystic button definitions
  public static final int A = 1;
  public static final int B = 2;
  public static final int X = 3;
  public static final int Y = 4;
  public static final int LEFT = 5;
  public static final int RIGHT = 6;
  public static final int BACK = 7;
  public static final int START = 8;
  public static final int LEFT_STICK = 9;
  public static final int RIGHT_STICK = 10;

  // home-grown PID coefficients
  public static final double kP = 0.2; 
  public static final double kI = 1e-4;
  public static final double kD = 1;

  public static final double kIz = 0; 
  public static final double kFF = 0; 
  public static final double kMaxOutput = 1; 
  public static final double kMinOutput = -1;

  String[] opModes = {"stop", "time", "distance", "pid_p", "pid_full", "xboxLeft"};

  public static final class steeringConstants {

    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);


    public static final int kFrontLeftsteeringMotorPort = 8;
    public static final int kBackLeftsteeringMotorPort = 2;
    public static final int kFrontRightsteeringMotorPort = 6;
    public static final int kBackRightsteeringMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 1;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 3;

    public static final boolean kFrontLeftsteeringMotorInverted = false;
    public static final boolean kBackLeftsteeringMotorInverted = false;
    public static final boolean kFrontRightsteeringMotorInverted = false;
    public static final boolean kBackRightsteeringMotorInverted = false;

    public static final boolean kFrontLeftsteeringEncoderInverted = false;
    public static final boolean kBackLeftsteeringEncoderInverted = false;
    public static final boolean kFrontRightsteeringEncoderInverted = false;
    public static final boolean kBackRightsteeringEncoderInverted = false;

    public static final int kFrontLeftsteeringAbsoluteEncoderPort = 0;
    public static final int kBackLeftsteeringAbsoluteEncoderPort = 2;
    public static final int kFrontRightsteeringAbsoluteEncoderPort = 1;
    public static final int kBackRightsteeringAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftsteeringAbsoluteEncoderInverted = false;
    public static final boolean kBackLeftsteeringAbsoluteEncoderInverted = false;
    public static final boolean kFrontRightsteeringAbsoluteEncoderInverted = false;
    public static final boolean kBackRightsteeringAbsoluteEncoderInverted = false;

    public static final double kFrontLeftsteeringAbsoluteEncoderOffsetRad = -0.254;
    public static final double kBackLeftsteeringAbsoluteEncoderOffsetRad = -1.252;
    public static final double kFrontRightsteeringAbsoluteEncoderOffsetRad = -1.816;
    public static final double kBackRightsteeringAbsoluteEncoderOffsetRad = -4.811;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTelesteeringMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTelesteeringMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    public static final double kTelesteeringMaxAccelerationUnitsPerSecond = 3;
    public static final double kTelesteeringMaxAngularAccelerationUnitsPerSecond = 3;
  }

    // swerve steering with TalonFx
  public static final class OffsetConstants {
    // distance to motors from the chasis center
    private static final double xLengthInInches = 23.; //in inches pointing upwards
    private static final double yLengthInInches = 23.; //in inches point to the left
    public static final double chasisXOffset = Units.inchesToMeters(xLengthInInches) / 2.;
    public static final double chasisYOffset = Units.inchesToMeters(yLengthInInches) / 2.;
  }
  // Team 5535 code
  public static final class SwerveMotorDeviceConstants {
    public static final int frontLeftSteeringId = 14;
    public static final int frontRightSteeringId = 15;
    public static final int backRightSteeringId = 16;
    public static final int backLeftSteeringId = 17;

    public static final int frontLeftDriveId = 20;
    public static final int frontRightDriveId = 21;
    public static final int backRightDriveId = 22;
    public static final int backLeftDriveId = 23;

    public static final int frontLeftCANcoderId = 10;
    public static final int frontRightCANcoderId = 11;
    public static final int backRightCANcoderId = 12;
    public static final int backLeftCANcoderId = 13;

    public static final double frontLeftAngleOffset = 358.;
    public static final double frontRightAngleOffset = 225.;
    public static final double backRightAngleOffset = 159.;
    public static final double backLeftAngleOffset = 250.;

    public static final boolean frontLeftdriveMotorInverted = false;
    public static final boolean backLeftdriveMotorInverted = false;
    public static final boolean frontRightdriveMotorInverted = false;
    public static final boolean backRightdriveMotorInverted = false;

    public static final boolean frontLeftsteeringMotorInverted = false;
    public static final boolean backLeftsteeringMotorInverted = false;
    public static final boolean frontRightsteeringMotorInverted = false;
    public static final boolean backRightsteeringMotorInverted = false;

  }

  public static final class SimulationSettings {

    public static final double AutonomousExampleSpeed0 = 0.3;
    public static final double AutonomousExampleTimer0 = 5.;
    public static final double AutonomousExampleSpeed1 = 0.4;
    public static final double AutonomousExampleTimer1 = 7.;

  }

  // 0 to Autonomous
      public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 5.8462;
      public static final double kTurningMotorGearRatio = 1 / 18.0;
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      public static final double kPTurning = 0.5;
  }
}
