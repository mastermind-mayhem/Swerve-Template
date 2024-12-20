// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

public class AutoCreationCmd {

  DrivetrainSubsystem drivetrain;

  /**
   * Method to autonomously drive the robot (ALL MEASUREMENTS IN METERS)
   *
   * @param _drivetrain Swerve Drivetrain Subsystem Instance
   * @param waypoints A list of points the robot should travel through
   *     <pre>List.of(new Translation2d(0, 1), new Translation2d(1, 1), new Translation2d(1, 0))
   *     </pre>
   *
   * @param finalrest Pose2d of what the robot's final resting position should be
   *     <pre>new Pose2d(x-offset, y-offset, new Rotation2d(rotationRadians))</pre>
   *
   * @return A Command variable telling the robot to drive
   */
  public Command AutoDriveCmd(
      DrivetrainSubsystem _drivetrain, List<Translation2d> waypoints, Pose2d finalrest) {
    drivetrain = _drivetrain;

    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    // Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0.01, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0.01, 0);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0.01, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Generate trajectory
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            drivetrain.getPose(), waypoints, finalrest, trajectoryConfig);

    // Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

    // Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> drivetrain.stopModules()));
  }

  public Command AutoDriveSpeedVar(
      Double maxSpeed,
      DrivetrainSubsystem _drivetrain,
      List<Translation2d> waypoints,
      Pose2d finalrest) {
    drivetrain = _drivetrain;

    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(maxSpeed, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    // Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0.01, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0.01, 0);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0.01, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // Generate trajectory
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            drivetrain.getPose(), waypoints, finalrest, trajectoryConfig);

    // Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

    // Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> drivetrain.stopModules()));
  }
}
