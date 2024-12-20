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

public class FourNoteAuto extends ParallelCommandGroup {

  public FourNoteAuto(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      FeederSubsystem feeder,
      ShooterSubsystem shooter,
      DashboardSubsystem dash) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command showyDrive1 =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(.08, 0)), new Pose2d(.2, 0, new Rotation2d(0)));
    Command showyDrive2 =
        autodrive.AutoDriveCmd(
            drivetrain, List.of(new Translation2d(.85, 0)), new Pose2d(1.69, 0, new Rotation2d(0)));
    Command showyDrive3 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-.85, dash.getAlliance() * 0.01)),
            new Pose2d(-1.60, 0, new Rotation2d(0)));
    Command showyDrive4 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(.5, dash.getAlliance() * 1)),
            new Pose2d(0.89, dash.getAlliance() * 1.3, new Rotation2d(0)));
    Command showyDrive5 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-.5, dash.getAlliance() * -1)),
            new Pose2d(-0.84, dash.getAlliance() * -1.44, new Rotation2d(0)));
    Command showyDrive6 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(.5, dash.getAlliance() * -1)),
            new Pose2d(0.89, dash.getAlliance() * -1.44, new Rotation2d(0)));
    Command showyDrive7 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Translation2d(-.5, dash.getAlliance() * 1)),
            new Pose2d(-0.84, dash.getAlliance() * 1.30, new Rotation2d(0)));
    addCommands(

        // Driving groups
        new SequentialCommandGroup(
            showyDrive1,
            new AutoSleepCmd(1),
            showyDrive2,
            new AutoSleepCmd(0),
            showyDrive3,
            new AutoSleepCmd(.5),
            showyDrive4,
            new AutoSleepCmd(0),
            showyDrive5,
            new AutoSleepCmd(.5),
            showyDrive6,
            new AutoSleepCmd(0),
            showyDrive7),

        // Intake group
        new SequentialCommandGroup(new AutoIntakeCmd(intake, 20)),

        // Feeder group
        new SequentialCommandGroup(
            new AutoSleepCmd(1.4), new AutoFeederCmd(feeder, true, .5),
            new AutoSleepCmd(2.9), new AutoFeederCmd(feeder, true, .5),
            new AutoSleepCmd(3.9), new AutoFeederCmd(feeder, true, .8),
            new AutoSleepCmd(3.85), new AutoFeederCmd(feeder, true, .8)),

        // 9.55
        // Shooter group
        new SequentialCommandGroup(
            new AutoSleepCmd(.5), new AutoShooterCmd(shooter, 1, 0.9, 1.5),
            new AutoSleepCmd(2.3), new AutoShooterCmd(shooter, 1, 0.9, 1.5),
            new AutoSleepCmd(3.8), new AutoShooterCmd(shooter, 1, 0.9, 1.8),
            new AutoSleepCmd(3.2), new AutoShooterCmd(shooter, 1, 0.9, 1.8)));
  }
}
