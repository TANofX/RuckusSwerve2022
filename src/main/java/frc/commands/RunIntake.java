// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  private DriverStation.Alliance currentAlliance;
  private Timer rejectTimer;
  private boolean checkReject;

  /** Creates a new RunIntake. */
  public RunIntake() {
    addRequirements(Intake.getInstance());
    rejectTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAlliance = DriverStation.getAlliance();
    checkReject = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (checkReject) {
      if (rejectTimer.hasElapsed(Constants.REVERSE_INTAKE_TIMEOUT)) {
        checkReject = false;
        Intake.getInstance().runIntake();
      }
    }
    else {
      if (BallHandler.getInstance().readyForIntake()) {
        Intake.getInstance().runIntake();
      }
      else {
        Intake.getInstance().stopIntake();
      }
      if (!Intake.getInstance().checkColor(currentAlliance)) {
        Intake.getInstance().reverseIntake();
        checkReject = true;
        rejectTimer.reset();
        rejectTimer.start();
      }
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().stopIntake();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
