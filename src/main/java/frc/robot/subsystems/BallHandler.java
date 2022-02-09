// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commands.DefaultBallHandler;
import frc.robot.Constants;
import frc.robot.util.BallSensor;

public class BallHandler extends SubsystemBase {
  public enum HandlerState {
    EMPTY,
    ONEBALLPOSITION1,
    ONEBALLPOSITION2,
    ONEBALLPOSITION3,
    ONEBALLPOSITION4,
    TWOBALLPOSITION1,
    TWOBALLPOSITION2,
    TWOBALLPOSITION3,
    UNKNOWN,
  }

  /** Creates a new BallHandler. */
  private Spark transitMotor;
  private BallSensor firstSensor; // far left (near intake)
  private BallSensor secondSensor;
  private BallSensor thirdSensor;
  private BallSensor fourthSensor; // far Right(near shooter)
  private HandlerState currentState = HandlerState.ONEBALLPOSITION1;
  private static BallHandler handlerInstance;
  private HandlerState targetState = HandlerState.ONEBALLPOSITION2;

  public BallHandler() {

    firstSensor = new BallSensor(Constants.HANDLERSENSOR_1_PORT);
    secondSensor = new BallSensor(Constants.HANDLERSENSOR_2_PORT);
    thirdSensor = new BallSensor(Constants.HANDLERSENSOR_3_PORT);
    fourthSensor = new BallSensor(Constants.HANDLERSENSOR_4_PORT);
    setDefaultCommand(new DefaultBallHandler());

  }

  /**
   * needs to extend (intake bar)
   * stop and start intake bar (spinning)
   * intake bar needs to spin in both directions (avoid jams)
   * start and stop lower wheels involved with intake/storage
   * need transit motors and control in both directions
   * Reverse for getting rid of ball(what has to be reversed)
   * 1 Neo motor for intake only -Spark = controller
   * 1 Neo motors for transit (intake)
   * 2 pistons to bring intake up and down
   * Retractable intake bar
   * color sensor, 4? sensors to detect ball
   * QUESTIONS:
   * How independent are the motors?
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateMachine();
    SmartDashboard.putString("HandlerState", currentState.name());
  }

  public void moveTransitMotor(double velocity) {
    transitMotor.set(velocity);
  }

  public void extake() {
    transitMotor.set(-0.25);
  }

  public void stopTransitMotor() {
    transitMotor.set(0);
  }

  public int ballsInRobot() {
    int numberOfBalls = 0;

    if (firstSensor.isTriggered()) {
      numberOfBalls = numberOfBalls + 1;
    }

    if (secondSensor.isTriggered()) {
      numberOfBalls = numberOfBalls + 1;
    }

    if (thirdSensor.isTriggered()) {
      numberOfBalls = numberOfBalls + 1;
    }

    if (fourthSensor.isTriggered()) {
      numberOfBalls = numberOfBalls + 1;
    }
    return numberOfBalls;

  }

  public boolean readyForIntake() {
    switch (currentState) {
      case EMPTY:
      case ONEBALLPOSITION2:

        return true;
      default:
        return false;
    }

  }

  public HandlerState getState() {
    return currentState;
  }

  public void setState(HandlerState newState) {
    targetState = newState;
    if (atState())
      return;
    switch (targetState) {
      case ONEBALLPOSITION2:
        switch (currentState) {
          case ONEBALLPOSITION1:
            moveTransitMotor(0.3);
            break;
          case ONEBALLPOSITION3:
          case ONEBALLPOSITION4:
            moveTransitMotor(-0.3);
            break;
          default:

        }
      case TWOBALLPOSITION3:

        switch (currentState) {
          case TWOBALLPOSITION1:
          case TWOBALLPOSITION2:
            moveTransitMotor(0.3);
            break;
          default:

        }

      default:

    }
  }

  public boolean atState() {
    return (currentState == targetState);
  }

  private void stateMachine() {
    if (ballsInRobot() == 0) {
      currentState = HandlerState.EMPTY;
    }

    if ((ballsInRobot() == 1) && (firstSensor.isTriggered())) {
      currentState = HandlerState.ONEBALLPOSITION1;
    }

    if ((ballsInRobot() == 1) && (secondSensor.isTriggered())) {
      currentState = HandlerState.ONEBALLPOSITION2;
    }

    if ((ballsInRobot() == 1) && (thirdSensor.isTriggered())) {
      currentState = HandlerState.ONEBALLPOSITION3;
    }

    if ((ballsInRobot() == 1) && (fourthSensor.isTriggered())) {
      currentState = HandlerState.ONEBALLPOSITION4;
    }

    if ((ballsInRobot() == 2) && (firstSensor.isTriggered()) && (secondSensor.isTriggered())) {
      currentState = HandlerState.TWOBALLPOSITION1;
    }

    if ((ballsInRobot() == 2) && (secondSensor.isTriggered()) && (thirdSensor.isTriggered())) {
      currentState = HandlerState.TWOBALLPOSITION2;
    }

    if ((ballsInRobot() == 2) && (thirdSensor.isTriggered()) && (fourthSensor.isTriggered())) {
      currentState = HandlerState.TWOBALLPOSITION3;
    }
  }

  public static BallHandler getInstance() {
    if (handlerInstance == null) {
      handlerInstance = new BallHandler();
    }
    return handlerInstance;
  }
}
