// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BeginSleepCmd extends Command {
  private Timer timer = new Timer();
  private double sleepTime;
  private DrivetrainSubsystem drivetrain;
  /** Creates a new AutoSleepCmd. */
  public BeginSleepCmd(DrivetrainSubsystem drives, double timeSleep) {
    sleepTime = timeSleep;
    drivetrain = drives;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setWait();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drivetrain.setGo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retVal = false;
    double currTime = timer.get();
    if (currTime >= sleepTime) {
      retVal = true;
    } else {
      retVal = false;
    }

    return retVal;
  }
}
