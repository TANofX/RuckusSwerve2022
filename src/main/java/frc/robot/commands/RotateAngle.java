// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateAngle extends CommandBase {
  /** Creates a new RotateAngle. */
  DrivetrainSubsystem driveTrain;
  private double angle;
  public RotateAngle(DrivetrainSubsystem driveTrain, double angle) {
    this.driveTrain = driveTrain;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double direction = Math.signum(angle - driveTrain.getGyroscopeRotation().getDegrees());
    driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, direction * 0.25, driveTrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, driveTrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return Math.abs(angle - driveTrain.getGyroscopeRotation().getDegrees()) < 2.0;
  
  }
}
