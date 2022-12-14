// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallHandler.HandlerState;

public class ShootAll extends CommandBase {
  /** Creates a new ShootAll. */
  public ShootAll() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!BallHandler.getInstance().shooterActivation()) {
      return;
    }
    if (Shooter.getInstance().correctSpeed()) {
      BallHandler.getInstance().shoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.getInstance().stopShooter();
    BallHandler.getInstance().shooterMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!BallHandler.getInstance().shooterActivation()) {
      return true;
    }
    return (BallHandler.getInstance().getState() == HandlerState.EMPTY) && (Shooter.getInstance().correctSpeed());
   //return (Shooter.getInstance().correctSpeed() && (BallHandler.getInstance().getState() == HandlerState.EMPTY));
  }
}
