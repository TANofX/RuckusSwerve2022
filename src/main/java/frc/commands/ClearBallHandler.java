
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSpeeds;

public class ClearBallHandler extends CommandBase {
  /** Creates a new ClearBallHandler. */
  private Timer clearTimer;

  public ClearBallHandler() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(BallHandler.getInstance(), Shooter.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clearTimer = new Timer();
    clearTimer.reset();
    clearTimer.start();
    Shooter.getInstance().startShooter(ShooterSpeeds.LOWGOAL);
    BallHandler.getInstance().moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.getInstance().stopShooter();
    BallHandler.getInstance().stopTransitMotor();
    BallHandler.getInstance().reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clearTimer.hasElapsed(Constants.CLEAR_TIMEOUT);
  }
}
