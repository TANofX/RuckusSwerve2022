// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTime extends CommandBase {
  /** Creates a new DriveTime. */
  private double x_speed;
  private double y_speed;
  private double time;
  private DrivetrainSubsystem driveTrain;
  private Timer timer;

  public DriveTime(DrivetrainSubsystem driveTrain, double x_speed, double y_speed, double time) {
    this.driveTrain = driveTrain;
    this.x_speed = x_speed;
    this.y_speed = y_speed;
    this.time = time;
    timer = new Timer();

    addRequirements(driveTrain);

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
    driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, 0, driveTrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, driveTrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
