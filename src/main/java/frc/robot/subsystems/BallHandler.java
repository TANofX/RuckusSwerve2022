// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private CANSparkMax transitMotor;
  private BallSensor firstSensor; // far left (near intake)
  private BallSensor secondSensor;
  private BallSensor thirdSensor;
  private BallSensor fourthSensor; // far Right(near shooter)
  private HandlerState currentState = HandlerState.ONEBALLPOSITION1;
  private static BallHandler handlerInstance;
  private HandlerState targetState = HandlerState.ONEBALLPOSITION2;
  private boolean shooterMode = false;
  private int numberOfBalls = 0;

  public BallHandler() {

    firstSensor = new BallSensor(Constants.HANDLERSENSOR_1_PORT);
    secondSensor = new BallSensor(Constants.HANDLERSENSOR_2_PORT);
    thirdSensor = new BallSensor(Constants.HANDLERSENSOR_3_PORT);
    fourthSensor = new BallSensor(Constants.HANDLERSENSOR_4_PORT);
    // setDefaultCommand(new DefaultBallHandler());
    transitMotor = new CANSparkMax(Constants.TRANSIT_MOTOR_ID, MotorType.kBrushless);
    transitMotor.setSmartCurrentLimit(29, 10);
    transitMotor.setOpenLoopRampRate(0.5);

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
    SmartDashboard.putNumber("number of Cargo", this.ballsInRobot());
  }

  public void moveTransitMotor(double velocity) {
    transitMotor.set(velocity);
  }

  public void extake() {
    transitMotor.set(-Constants.TRANSIT_MOTOR_SPEED);
  }

  public void stopTransitMotor() {
    transitMotor.set(0);
  }

  public int ballsInRobot() {
    int internalNumberOfBalls = 0;

    if (firstSensor.isTriggered() && Intake.getInstance().isIntakeExtended()) {
      internalNumberOfBalls = internalNumberOfBalls + 1;
    }

    if (secondSensor.isTriggered()) {
      internalNumberOfBalls = internalNumberOfBalls + 1;
    }

    if (thirdSensor.isTriggered()) {
      internalNumberOfBalls = internalNumberOfBalls + 1;
    }

    if (fourthSensor.isTriggered()) {
      internalNumberOfBalls = internalNumberOfBalls + 1;
    }

    //if (internalNumberOfBalls > numberOfBalls) {
      numberOfBalls = internalNumberOfBalls;
    //}

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

  public void shooterMode(boolean activate) {
    shooterMode = activate;
  }

  public boolean shooterActivation() {
    return shooterMode;
  }

  public void setState(HandlerState newState) {
    targetState = newState;
    if (atState())
      return;
    switch (targetState) {
      case ONEBALLPOSITION2:
        switch (currentState) {
          case ONEBALLPOSITION1:
            moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
            break;
          case ONEBALLPOSITION3:
          case ONEBALLPOSITION4:
            moveTransitMotor(-Constants.TRANSIT_MOTOR_SPEED);
            break;
          default:

        }
      case TWOBALLPOSITION3:

        switch (currentState) {
          case TWOBALLPOSITION1:
          case TWOBALLPOSITION2:
            moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
            break;
          default:

        }
      case ONEBALLPOSITION4:
        switch (currentState) {
          case ONEBALLPOSITION1:
          case ONEBALLPOSITION2:
          case ONEBALLPOSITION3:
            moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
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
    HandlerState nextState = HandlerState.UNKNOWN;
    if (ballsInRobot() == 0) {
      nextState = HandlerState.EMPTY;
    }

    if ((ballsInRobot() == 1) && (firstSensor.isTriggered())) {
      nextState = HandlerState.ONEBALLPOSITION1;
    }

    if ((ballsInRobot() == 1) && (secondSensor.isTriggered())) {
      nextState = HandlerState.ONEBALLPOSITION2;
    }

    if ((ballsInRobot() == 1) && (thirdSensor.isTriggered())) {
      nextState = HandlerState.ONEBALLPOSITION3;
    }

    if ((ballsInRobot() == 1) && (fourthSensor.isTriggered())) {
      nextState = HandlerState.ONEBALLPOSITION4;
    }

    if ((ballsInRobot() == 2) && (firstSensor.isTriggered()) && (secondSensor.isTriggered())) {
      nextState = HandlerState.TWOBALLPOSITION1;
    }

    if ((ballsInRobot() == 2) && (secondSensor.isTriggered()) && (thirdSensor.isTriggered())) {
      nextState = HandlerState.TWOBALLPOSITION2;
    }

    if ((ballsInRobot() == 2) && (thirdSensor.isTriggered()) && (fourthSensor.isTriggered())) {
      nextState = HandlerState.TWOBALLPOSITION3;
    }

    SmartDashboard.putString("next state", nextState.name());

    switch (currentState) {
      case EMPTY:
        switch (nextState) {
          case ONEBALLPOSITION1:
          case ONEBALLPOSITION2:
          case ONEBALLPOSITION4:
            currentState = nextState;
            break;
          default:
        }
        break;
      case ONEBALLPOSITION1:
        switch (nextState) {
          case ONEBALLPOSITION2:
          case EMPTY:
            currentState = nextState;
            break;
          default:
        }
      case ONEBALLPOSITION2:
        switch (nextState) {
          case ONEBALLPOSITION3:
          case ONEBALLPOSITION1:
          case TWOBALLPOSITION1:
            currentState = nextState;
            break;
          default:
        }
      case ONEBALLPOSITION3:
        switch (nextState) {
          case ONEBALLPOSITION2:
          case ONEBALLPOSITION4:
            currentState = nextState;
            break;
          default:
        }
      case ONEBALLPOSITION4:
        switch (nextState) {
          case ONEBALLPOSITION3:
          case EMPTY:
            currentState = nextState;
            break;
          default:
        }
      case TWOBALLPOSITION1:
        switch (nextState) {
          case ONEBALLPOSITION2:
          case TWOBALLPOSITION2:
          case ONEBALLPOSITION1:
            currentState = nextState;
            break;
          default:
        }
      case TWOBALLPOSITION2:
        switch (nextState) {
          case TWOBALLPOSITION1:
          case TWOBALLPOSITION3:
            currentState = nextState;
            break;
          default:
        }
      case TWOBALLPOSITION3:
        switch (nextState) {
          case TWOBALLPOSITION2:
          case ONEBALLPOSITION4:
          case ONEBALLPOSITION3:
            currentState = nextState;
            break;
          default:
        }
      default:
      currentState = nextState;
    }
  }

  public static BallHandler getInstance() {
    if (handlerInstance == null) {
      handlerInstance = new BallHandler();
    }
    return handlerInstance;
  }
}
