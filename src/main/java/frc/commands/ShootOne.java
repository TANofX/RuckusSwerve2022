// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallHandler.HandlerState;

public class ShootOne extends CommandBase {
  private HandlerState targetState;
  private boolean shot;
  /** Creates a new ShootAll. */
  public ShootOne() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (BallHandler.getInstance().ballsInRobot() == 2) {
      targetState = HandlerState.ONEBALLREADYTOSHOOT;
    }
    else {
      targetState = HandlerState.EMPTY;
    }
    shot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (!BallHandler.getInstance().shooterActivation()) {
     return;
   }
    if (Shooter.getInstance().correctSpeed() && !shot) {
      BallHandler.getInstance().shoot();
      shot = true;
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
   return (Shooter.getInstance().correctSpeed() && (BallHandler.getInstance().getState() == targetState));
  }
}
