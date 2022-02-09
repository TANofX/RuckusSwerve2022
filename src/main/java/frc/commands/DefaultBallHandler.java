// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.BallHandler.HandlerState;

public class DefaultBallHandler extends CommandBase {
  /** Creates a new DefaultBallHandler. */
  public DefaultBallHandler() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(BallHandler.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (BallHandler.getInstance().getState()) {
      case ONEBALLPOSITION1:
      case ONEBALLPOSITION3:
      case ONEBALLPOSITION4:
        BallHandler.getInstance().setState(HandlerState.ONEBALLPOSITION2);
        break;
      case TWOBALLPOSITION1:
      case TWOBALLPOSITION2:
        BallHandler.getInstance().setState(HandlerState.TWOBALLPOSITION3);
        break;
      default:
        BallHandler.getInstance().stopTransitMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BallHandler.getInstance().stopTransitMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
