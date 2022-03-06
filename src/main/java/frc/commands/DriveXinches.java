// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveXinches extends CommandBase {
  /** Creates a new DriveXinches. */
  private double length;
  private double target;
  public DriveXinches(double distance) {
    length = distance * 0.0254;
    addRequirements(DriveSubsystem.getInstance());

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = length + DriveSubsystem.getInstance().getWheelPosition();
    SmartDashboard.putNumber("Target Position", target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = DriveSubsystem.getInstance().getWheelPosition();
    SmartDashboard.putNumber("Current Position", currentPosition);
    double difference = target - currentPosition;
    DriveSubsystem.getInstance().curvatureDrive(.10 * Math.signum(difference) + 0.35 * difference, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double difference = target - DriveSubsystem.getInstance().getWheelPosition();
    if (Math.abs(difference) < 0.1) {
      return true;
    }
    return false;
  }
}
