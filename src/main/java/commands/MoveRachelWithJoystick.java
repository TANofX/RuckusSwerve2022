// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class MoveRachelWithJoystick extends CommandBase {
  /** Creates a new MoveRachelWithJoystick. */
  Climber climber;
  Joystick joystick;
  double lastExecuteTime;
  List<Double> frameTimes;


  public MoveRachelWithJoystick(Climber rachel, Joystick flight) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = rachel;
    joystick = flight;
    frameTimes = new ArrayList<Double>();
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastExecuteTime = RobotController.getFPGATime() / 1000.0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentFrameTime = RobotController.getFPGATime() / 1000.0;
    double elapsedFrameTime = currentFrameTime - lastExecuteTime;
    frameTimes.add(elapsedFrameTime);
    lastExecuteTime = currentFrameTime;
    double averageFrameTime = frameTimes.stream().mapToDouble(d->d).average().orElse(0.0);

    double currentPosition = climber.getRachelPosition();
    double targetVelocity = climber.getMaxVelicity() * joystick.getRawAxis(Constants.MOVE_RACHEL_AXIS);
    double targetPosition = currentPosition + targetVelocity * averageFrameTime * 2;
    
    climber.moveRachelPosition(targetPosition);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopRachel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
