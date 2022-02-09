// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;

public class SetClimberState extends CommandBase {
  /** Creates a new SetClimberState. */
  private ClimberState goalState;
  public SetClimberState(ClimberState targeState) {
    goalState = targeState;
    addRequirements(Climber.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climber.getInstance().goToTargetState(goalState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Climber.getInstance().getCurrentState() == goalState);
  }
}
