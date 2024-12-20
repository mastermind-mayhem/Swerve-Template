// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.utils.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          RobotConstants.kFrontLeftDrivingCanId,
          RobotConstants.kFrontLeftTurningCanId,
          RobotConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          RobotConstants.kFrontRightDrivingCanId,
          RobotConstants.kFrontRightTurningCanId,
          RobotConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          RobotConstants.kRearLeftDrivingCanId,
          RobotConstants.kRearLeftTurningCanId,
          RobotConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          RobotConstants.kRearRightDrivingCanId,
          RobotConstants.kRearRightTurningCanId,
          RobotConstants.kBackRightChassisAngularOffset);

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // The gyro sensor
  private AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double x;
  private double y;
  private double r;

  private boolean waiting = false;
  private double maxSpeedDrive;
  private double maxSpeedTurn;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_magLimiter1 = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    zeroHeading();
    m_gyro.calibrate();

    m_chooser.setDefaultOption("Medium Speed", DriveConstants.medium);
    m_chooser.addOption("Low Speed", DriveConstants.low);
    m_chooser.addOption("High Speed", DriveConstants.high);
    SmartDashboard.putData("Speed Drop Down", m_chooser);
  }

  /**
   * Method to drive the robot using joystick info. (Field Oriented)
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param maxTurn Max angular speed.
   * @param maxDrive Max driving speed.
   */
  public void fieldDrive(
      double xSpeed, double ySpeed, double rot, double maxTurn, double maxDrive) {
    maxSpeedDrive = maxDrive;
    maxSpeedTurn = maxTurn;

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = m_magLimiter.calculate(xSpeed);
    ySpeedCommanded = m_magLimiter1.calculate(ySpeed);
    m_currentRotation = m_rotLimiter.calculate(rot);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeedDrive;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedDrive;
    double rotDelivered = m_currentRotation * maxSpeedTurn;

    x = xSpeedDelivered;
    y = ySpeedDelivered;
    r = rotDelivered;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info. (Robot Oriented)
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param maxTurn Max angular speed.
   * @param maxDrive Max driving speed.
   */
  public void robotDrive(
      double xSpeed, double ySpeed, double rot, double maxTurn, double maxDrive) {
    maxSpeedDrive = maxDrive;
    maxSpeedTurn = maxTurn;

    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeedCommanded = m_magLimiter.calculate(xSpeed);
    ySpeedCommanded = m_magLimiter1.calculate(ySpeed);
    m_currentRotation = m_rotLimiter.calculate(rot);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeedDrive;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedDrive;
    double rotDelivered = m_currentRotation * maxSpeedTurn;

    x = xSpeedDelivered;
    y = ySpeedDelivered;
    r = rotDelivered;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  // Not used could be applied later though
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
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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

  /** Zeroes the heading of the robot. */
  public double gyroangle() {
    return m_gyro.getAngle() * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getHeading() {

    return Math.IEEEremainder(m_gyro.getAngle() * (RobotConstants.kGyroReversed ? -1.0 : 1.0), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
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
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  public String getDropDown() {
    return m_chooser.getSelected();
  }

  public void setWait() {
    waiting = true;
  }

  public void setGo() {
    waiting = false;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Z axis angle", getHeading());
    SmartDashboard.putBoolean("Auto is Waiting", waiting);

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }
}
