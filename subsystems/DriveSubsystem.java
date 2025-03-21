// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// Library Dependecies
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.utils.LimelightHelpers;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  public boolean rotateBool = false;
  public static boolean balanceBool = false;
  private static double kMaxSpeedMetersPerSecond = 3;
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Range sensor
  // private TimeOfFlight m_rangeSensor = new TimeOfFlight(1);

  // The gyro sensor
  
  public final static AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    

    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Camera", 640, 480);

  //   RobotConfig config = null;
  //       try{
  //         config = RobotConfig.fromGUISettings();
  //       } catch (Exception e) {
  //        // Handle exception as needed
  //        e.printStackTrace();
  //      }
  //   // Configure AutoBuilder last
  //   AutoBuilder.configure(
  //           this::getPose, // Robot pose supplier
  //           this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
  //           this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //           (speeds) -> setSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
  //           new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
  //                   new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  //                   new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
  //           ),
  //           config, // The robot configuration
  //           () -> {
  //             // Boolean supplier that controls when the path will be mirrored for the red alliance
  //             // This will flip the path being followed to the red side of the field.
  //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //             var alliance = DriverStation.getAlliance();
  //             if (alliance.isPresent()) {
  //               return alliance.get() == DriverStation.Alliance.Red;
  //             }
  //             return false;
  //           },
  //           this // Reference to this subsystem to set requirements
  //   );
   }

  // public ChassisSpeeds getSpeeds() {
  //   return speeds;
  // }

  // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
  //     0, .0, 0, Rotation2d.fromDegrees(0.0));

  // public void setSpeeds(ChassisSpeeds speeds) {
  //   System.out.println(speeds);
  //   this.speeds = speeds;
    
  //   if (this.speeds.omegaRadiansPerSecond >= -.1 && this.speeds.omegaRadiansPerSecond <= .1) {
  //     drive(this.speeds.vxMetersPerSecond, this.speeds.vyMetersPerSecond, 0, true, false);
  //   } else {
  //     if (this.speeds.omegaRadiansPerSecond < 0) {
  //       this.speeds.omegaRadiansPerSecond = -this.speeds.omegaRadiansPerSecond;
  //     }
  //     drive(this.speeds.vxMetersPerSecond, this.speeds.vyMetersPerSecond, this.speeds.omegaRadiansPerSecond* .04, true, false);
  //   }
  //   drive(this.speeds.vxMetersPerSecond, this.speeds.vyMetersPerSecond, this.speeds.omegaRadiansPerSecond, false, false);
  // }

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  @Override
  public void periodic() {
    try {
      getHeading();
    } catch (Exception e) {
      System.out.println("get heading failed, exception: " + e);
    }
    SmartDashboard.putNumber("Gyro Rotation", getHeading());

    try {
      m_odometry.update(
          Rotation2d.fromDegrees(m_gyro.getAngle()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });
    } catch (Exception e) {
      System.out.println("odometry failed, exception: " + e);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() - .1),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle() * -1))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        (SwerveModuleState[]) swerveModuleStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void StopAtAngle(int angle) {
    double rotationSpeed;
    if (angle > m_gyro.getAngle()) {
      rotationSpeed = -.1;
    } else {
      rotationSpeed = .1;
    }
    if (m_gyro.getAngle() <= angle + 5 && m_gyro.getAngle() >= angle - 5) {
      rotateBool = true;
    } else {
      drive(0, 0, rotationSpeed, true, false);
      rotateBool = false;
    }
  }

  public void StopAtPosition(Double posX, double posY) {
    double tempXSpeed;
    double tempYSpeed;
    if (getPose().getX() <= posX + .1 && getPose().getX() >= posX - .1) {
      tempXSpeed = 0;
    } else {
      tempXSpeed = .1;
    }
    if (getPose().getY() <= posY + .1 && getPose().getY() >= posY - .1) {
      tempYSpeed = 0;
    } else {
      tempYSpeed = .1;
    }

    drive(tempXSpeed, tempYSpeed, 0, true, false);
  }
  

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void RotateToTarget() {
    int[] validIDs = {-1, 0, 14, 5, 6};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
     double x = LimelightHelpers.getTX("");
     // double rotation = m_gyro.getAngle();
     drive(.001,.001,-x * .01, false, false);
     System.out.println(x);
     System.out.println("Id: " + LimelightHelpers.getFiducialID("limelight"));
    }

}