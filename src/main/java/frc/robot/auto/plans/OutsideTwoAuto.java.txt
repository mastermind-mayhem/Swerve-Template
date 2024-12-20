// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoFeederCmd;
import frc.robot.auto.AutoIntakeCmd;
import frc.robot.auto.AutoShooterCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;

public class OutsideTwoAuto extends ParallelCommandGroup {
  public OutsideTwoAuto(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      FeederSubsystem feeder,
      ShooterSubsystem shooter,
      DashboardSubsystem dash) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands
    Command RightShoot =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(0.3, 0)),
            new Pose2d(
                0.76,
                dash.getAlliance() * 0.12,
                new Rotation2d(dash.getAlliance() * -2 * Math.PI / 3)));

    Command RightNote =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(0.156, dash.getAlliance() * 0.6)),
            new Pose2d(0.95, dash.getAlliance() * 1.53, new Rotation2d(0)));

    Command ReturnNote =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-.95 / 2, dash.getAlliance() * -1.53 / 2)),
            new Pose2d(-.95, dash.getAlliance() * -1.53, new Rotation2d(0)));

    Command Taxi =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(1, 1)),
            new Pose2d(1.2, dash.getAlliance() * 1.2, new Rotation2d(0)));

    addCommands(
        // Drivetrain Sequential
        new SequentialCommandGroup(
            RightShoot,
            new AutoSleepCmd(.5),
            RightNote,
            new AutoSleepCmd(.5),
            ReturnNote,
            new AutoSleepCmd(.5),
            Taxi),

        // Intake Sequential
        new SequentialCommandGroup(new AutoSleepCmd(0), new AutoIntakeCmd(intake, 15)),

        // Feeder Sequential
        new SequentialCommandGroup(
            new AutoSleepCmd(1),
            new AutoFeederCmd(feeder, true, 0.5),
            new AutoSleepCmd(4.5),
            new AutoFeederCmd(feeder, true, .5)),

        // Shooter Sequential
        new SequentialCommandGroup(
            new AutoShooterCmd(shooter, 1, 0.85, 2),
            new AutoSleepCmd(3),
            new AutoShooterCmd(shooter, 1, 0.85, 2)));
  }
}
