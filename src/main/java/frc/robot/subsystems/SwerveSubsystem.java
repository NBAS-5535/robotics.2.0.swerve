package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OffsetConstants;
import frc.robot.Constants.SwerveMotorDeviceConstants;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;


class SwerveModule {
  // Motors
  private final SparkFlex driveMotor;
  private final SparkFlex steeringMotor;

  private final boolean driveMotorInverted;
  private final boolean steeringMotorInverted;
  // Encoders
  //private final CANcoder driveEncoder;
  //private final CANcoder steeringEncoder;
  private final CANcoder angleEncoder;
  private final boolean angleEncoderInverted;
  

  // Offsets
  private final double angleOffset;

  private final PIDController steeringPidController;
  private SwerveModuleState currentState;


  public SwerveModule(int driveMotorPort, int steeringMotorPort,
                      boolean driveMotorInverted, boolean steeringMotorInverted,
                      int encoderPort, double encoderOffset, boolean encoderInverted) {
      /**/
      System.out.print("Ctor - SwerveModule: drive/steering Port# ");
      System.out.print(driveMotorPort);
      System.out.print(" / ");
      System.out.println(steeringMotorPort);
      /**/
      
      this.driveMotor = new SparkFlex(driveMotorPort, MotorType.kBrushless);
      this.steeringMotor = new SparkFlex(steeringMotorPort, MotorType.kBrushless);

      this.driveMotorInverted = driveMotorInverted;
      this.steeringMotorInverted = steeringMotorInverted;

      //driveMotor.setInverted(driveMotorInverted);
      //steeringMotor.setInverted(steeringMotorInverted);
      
      this.angleEncoder = new CANcoder(encoderPort);
      this.angleEncoderInverted = encoderInverted;

      this.angleOffset = encoderOffset;

      this.steeringPidController = new PIDController(ModuleConstants.kPTurning, 0., 0.);
      steeringPidController.enableContinuousInput(-Math.PI, Math.PI);

      
      currentState = new SwerveModuleState();

      //driveMotor.setInverted(true);
  }

  public SwerveModuleState getState(){
      return currentState;
  }

  public void setState(SwerveModuleState newState){
      currentState = newState;
  }

  public void stop() {
      driveMotor.set(0);
      steeringMotor.set(0);
  }

  public void setTestSpeed(double drive, double turn) {
    driveMotor.set(drive);
    steeringMotor.set(turn);
}
}

public class SwerveSubsystem extends SubsystemBase {
    //int driveMotorPort;
    //int steeringMotorPort;

    private final SwerveModule m_frontLeftModule   = new SwerveModule(SwerveMotorDeviceConstants.frontLeftDriveId, SwerveMotorDeviceConstants.frontLeftSteeringId, SwerveMotorDeviceConstants.frontLeftdriveMotorInverted, SwerveMotorDeviceConstants.frontLeftsteeringMotorInverted, SwerveMotorDeviceConstants.frontLeftCANcoderId, SwerveMotorDeviceConstants.frontLeftAngleOffset, SwerveMotorDeviceConstants.frontLeftdriveMotorInverted);
    private final SwerveModule m_frontRightModule  = new SwerveModule(SwerveMotorDeviceConstants.frontRightDriveId, SwerveMotorDeviceConstants.frontRightSteeringId,
                                                        SwerveMotorDeviceConstants.frontRightdriveMotorInverted, SwerveMotorDeviceConstants.frontRightsteeringMotorInverted,
                                                        SwerveMotorDeviceConstants.frontRightCANcoderId, SwerveMotorDeviceConstants.frontRightAngleOffset,
                                                        SwerveMotorDeviceConstants.frontRightdriveMotorInverted);
    private final SwerveModule m_backLeftModule    = new SwerveModule(SwerveMotorDeviceConstants.backLeftDriveId, SwerveMotorDeviceConstants.backLeftSteeringId,
                                                        SwerveMotorDeviceConstants.backLeftdriveMotorInverted, SwerveMotorDeviceConstants.backLeftsteeringMotorInverted,
                                                        SwerveMotorDeviceConstants.backLeftCANcoderId, SwerveMotorDeviceConstants.backLeftAngleOffset,
                                                        SwerveMotorDeviceConstants.backLeftdriveMotorInverted);
    private final SwerveModule m_backRightModule   = new SwerveModule(SwerveMotorDeviceConstants.backRightDriveId, SwerveMotorDeviceConstants.backRightSteeringId,
                                                        SwerveMotorDeviceConstants.backRightdriveMotorInverted, SwerveMotorDeviceConstants.backRightsteeringMotorInverted,
                                                        SwerveMotorDeviceConstants.backRightCANcoderId, SwerveMotorDeviceConstants.backRightAngleOffset,
                                                        SwerveMotorDeviceConstants.backRightdriveMotorInverted);

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation  = new Translation2d( OffsetConstants.chasisXOffset,  OffsetConstants.chasisYOffset);
    Translation2d m_frontRightLocation = new Translation2d( OffsetConstants.chasisXOffset, -OffsetConstants.chasisYOffset);
    Translation2d m_backLeftLocation   = new Translation2d(-OffsetConstants.chasisXOffset,  OffsetConstants.chasisYOffset);
    Translation2d m_backRightLocation  = new Translation2d(-OffsetConstants.chasisXOffset, -OffsetConstants.chasisYOffset);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, 
                                                                   m_backLeftLocation, m_backRightLocation);

    // reference to controller
    CommandXboxController m_controller;
    
    public SwerveSubsystem(CommandXboxController driverController) {
        System.out.println("Ctor- SwerveSubsytem");
        this.m_controller = driverController;
    }

    @Override
    public void periodic() {
        /*
        // read data from the controller
        // get x- and y-values of the left joystick
        // Chasis speeds are (dx, dy, theta)
        ChassisSpeeds newSpeed = new ChassisSpeeds(
            m_controller.getRightY(), // push forward: robot moves forward
            m_controller.getLeftX(), // push left: robot moves left
            m_controller.getRightX()  // push left: robot rotates left
        );
      
        //newSpeed = new ChassisSpeeds(0., 1., 0.);
        System.out.println(newSpeed);
        // pass Chassis speed to get module data
        setChasisSpeeds(newSpeed);
        */

        // current state
        //System.out.println(getCurrentChasisSpeeds());
        //SmartDashboard.putNumberArray("Current Swerve states", getCurrentChasisSpeeds());
        showChasisSpeedsOnLogger("periodic");
        //the order FL, FR, BL, BR
        double loggingState[] = {m_frontLeftModule.getState().angle.getDegrees(),
                                 m_frontLeftModule.getState().speedMetersPerSecond,
                                 m_frontRightModule.getState().angle.getDegrees(),
                                 m_frontRightModule.getState().speedMetersPerSecond,
                                 m_backLeftModule.getState().angle.getDegrees(),
                                 m_backLeftModule.getState().speedMetersPerSecond,
                                 m_backRightModule.getState().angle.getDegrees(),
                                 m_backRightModule.getState().speedMetersPerSecond
        };
        //double loggingState[] = {0, 1., 45., 2., 90., 3., 135., 4.};
        SmartDashboard.putNumberArray("Output Swerve states", loggingState);
    }
    
    public void setChasisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds); 

        m_frontLeftModule.setState(moduleStates[0]);
        m_frontRightModule.setState(moduleStates[1]);
        m_backLeftModule.setState(moduleStates[2]);
        m_backRightModule.setState(moduleStates[3]);
    }

    public double[] getCurrentChasisSpeeds() {
      double loggingState[] = {m_frontLeftModule.getState().angle.getDegrees(),
        m_frontLeftModule.getState().speedMetersPerSecond,
        m_frontRightModule.getState().angle.getDegrees(),
        m_frontRightModule.getState().speedMetersPerSecond,
        m_backLeftModule.getState().angle.getDegrees(),
        m_backLeftModule.getState().speedMetersPerSecond,
        m_backRightModule.getState().angle.getDegrees(),
        m_backRightModule.getState().speedMetersPerSecond};

        return loggingState;
    }

    public void stopModules() {
        m_frontLeftModule.stop();
        m_frontRightModule.stop();
        m_backLeftModule.stop();
        m_backRightModule.stop();
    }

    public void setModuleTestSpeed(double drive, double turn) {
      m_frontLeftModule.setTestSpeed(drive, turn);
      m_frontRightModule.setTestSpeed(drive, turn);
      m_backLeftModule.setTestSpeed(drive, turn);
      m_backRightModule.setTestSpeed(drive, turn);
  }

    public void showChasisSpeedsOnLogger(String labelString){
      double currentState[] = getCurrentChasisSpeeds();
      for ( int i = 0; i < currentState.length; i++) {
        String output = labelString + "State:" + Integer.toString(i);
        //SmartDashboard.putNumber("State:" + Integer.toString(i), currentState[i]);
        SmartDashboard.putNumber(output, currentState[i]);
        if ( Math.abs(currentState[i]) > 0.0 ) {
          if ( 1 == 0 ){
            System.out.print("State:[" + Integer.toString(i) + "]: " + currentState[i] + " / ");
            if (i == currentState.length - 1) {
              System.out.println();
            }
          }
        }
      }
    }

    ////////////////////////////////////
    // my methods
    public double averageEncoderPosition(){
        //System.out.print("Ctor - SwerveModule: averageEncoderPosition ");
        return 0.;
    }
    
    // maybe better to move back to robotcontainer????
    public double runTimeBasedControlLogic(double maxTime, double startTime, double outputSpeed) {
    // get the current time and check the diff
    double currentTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("currentTime", currentTime);
    SmartDashboard.putNumber("timeDiff", currentTime - startTime);
    if (currentTime - startTime > maxTime) {
      outputSpeed = 0.;
      //SmartDashboard.putString("Stop", "Stop");
      SmartDashboard.putString("Reason_to_Stop", "maxTime reached");
    }
    //System.out.print("Ctor - SwerveModule: runTimeBasedControlLogic ");
    SmartDashboard.putNumber("outputSpeed", outputSpeed);
    return outputSpeed;
  }

  public double runDistanceBasedControlLogic(double maxDistance, double startDistance, double currentDistance, double outputSpeed) {
    if (currentDistance - startDistance > maxDistance) {
      outputSpeed = 0.;
      SmartDashboard.putString("Stop", "Stop");
      SmartDashboard.putString("Reason_to_Stop", "maxDistance reached");
    }
    return outputSpeed;
  }

  public double runPIDBasedControlLogic(double distance2Travel, double currentDistance, double kP, double outputSpeed) {
    double distanceLeftToGo = distance2Travel - currentDistance;
    //outputSpeed = kP * distanceLeftToGo / distance2Travel;
    outputSpeed *= distanceLeftToGo / distance2Travel;
    SmartDashboard.putNumber("newSpeed", outputSpeed);
    if (outputSpeed < Constants.epsilonPower || distanceLeftToGo < 0.01) {
      outputSpeed = 0.;
      SmartDashboard.putString("Stop", "Stop");
      SmartDashboard.putString("Reason_to_Stop", "speed < epsilon");
    }
    return outputSpeed;
  }
  
  public void moveRobot(double outputSpeed){
  }

  public void turnRobot(double outputSpeed){

  }

    public final void setPIDConstants(SparkClosedLoopController m_pidController, 
                                    double kP, double kI, double kD, double kIz, double kFF,
                                    double kMinOutput, double kMaxOutput){
 
      // set PID coefficients

    }
}
