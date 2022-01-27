// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.RunnableScheduledFuture;
import java.util.spi.CurrencyNameProvider;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
      //Rachel Falcons to extend and retract climber bars
      private WPI_TalonFX leftRachelFalcon;
      private WPI_TalonFX rightRachelFalcon;
      
      //passed through motors so we can read them through there
      //Up and Down limits for the extending bars.  These will only be used to calibrate and as a fail safe if code tells them to go past their point of no return
      private DigitalInput rachelUpLeftLimit;
      private DigitalInput rachelDownLeftLimit;
      private DigitalInput rachelUpRightLimit;
      private DigitalInput rachelDownRightLimit;

      //Pistons for Gabe's active claw and Rachel's Reach function
      private Solenoid gabeLeftClaw;
      private Solenoid gabeRightClaw;
      private Solenoid rachelLeftReach;
      private Solenoid rachelRightReach;

      //Limit switches which will allow us to know if we are clawed on a bar for gabe or rachel
      private DigitalInput gabeLeftIdentification;
      private DigitalInput gabeRightIndentification;
      private DigitalInput rachelLeftBarSensor;
      private DigitalInput rachelRightBarSensor;

      public static enum ClimberState {
            UNKNOWN, STARTING_CONFIG, REACHING, BABYS_FIRST_REACH, BABYS_FIRST_PULL_UP, SUCESSFUL_PULL_UP, SUCESSFUL_HANG, RELEASE_REACH, T_REX_REACH, STEGOSAURUS_REACHING, BRONTOSAURUS_REACHING, REACH_PULL, REACH_SEARCHING, REACH_CATCH, REACH_CAUGHT, TRUST_FALL
      }

      private static enum GabeStates {
            UNKNOWN,
            OPEN,
            CLOSED_WITH_BAR,
            CLOSED_WITHOUT_BAR
      }

      // None of the "expectecPosition" values are correct. They will need to be determined experimentally
      private static enum RachelExtensionStates {
            FULLY_RETRACTED(0),
            GABE_HEIGHT(1000),
            FULLY_EXTENDED(6000),
            EXPECTED_CATCH_LOCATION(5000),
            REACH_EXTENSION(5500),
            FIRST_REACH_EXTENSION(4500),
            UNKNOWN(-100000),
            EXTENDING(-100001),
            RETRACTING(-100002);

            private double expectedPosition;
            private static double allowedPositionError = 10.0;

            private RachelExtensionStates(double encoderPosition) {
                  expectedPosition = encoderPosition;
            }

            private double getMotorTarget() {
                  return expectedPosition;
            }

            private static RachelExtensionStates findState(double position) {
                  for (RachelExtensionStates s: RachelExtensionStates.values()) {
                        if (Math.abs(s.getMotorTarget() - position) < allowedPositionError) {
                              return s;
                        }
                  }

                  return UNKNOWN;
            }
      }

      private static enum RachelBarStates {
            UNKNOWN,
            NO_BAR,
            BAR_CONTACT
      }

      private ClimberState currentState;

  public Climber() {



      leftRachelFalcon = new WPI_TalonFX(Constants.LEFT_RACHEL_FALCON);
      rightRachelFalcon = new WPI_TalonFX(Constants.RIGHT_RACHEL_FALCON);
      //talon.getSensorCollection().isRevLimitSwitchClosed() == 1=closed and 0=open

      rachelUpLeftLimit = new DigitalInput(Constants.RACHEL_UP_LEFT_LIMIT);
      rachelDownLeftLimit = new DigitalInput(Constants.RACHEL_DOWN_LEFT_LIMIT);
      rachelUpRightLimit = new DigitalInput(Constants.RACHEL_UP_RIGHT_LIMIT);
      rachelDownRightLimit = new DigitalInput(Constants.RACHEL_DOWN_RIGHT_LIMIT);

      //What should the pneumatics module type be? 
      gabeLeftClaw = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.GABE_LEFT_CLAW);
      gabeRightClaw = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.GABE_RIGHT_CLAW);
      rachelLeftReach = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.RACHEL_LEFT_REACH);
      rachelRightReach = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.RACHEL_RIGHT_REACH);

      gabeLeftIdentification = new DigitalInput(Constants.GABE_LEFT_IDENTIFICATION);
      gabeRightIndentification = new DigitalInput(Constants.GABE_RIGHT_IDENTIFICATION);
      rachelLeftBarSensor = new DigitalInput(Constants.RACHEL_LEFT_BAR_SENSOR);
      rachelRightBarSensor = new DigitalInput(Constants.RACHEL_RIGHT_BAR_SENSOR);

  }

  private RachelBarStates getRachelBarState() {
        if (rachelLeftBarSensor.get() && rachelRightBarSensor.get()) {
              return RachelBarStates.BAR_CONTACT;
        } else if (!rachelLeftBarSensor.get() && !rachelRightBarSensor.get()) {
              return RachelBarStates.NO_BAR;
        }

        return RachelBarStates.UNKNOWN;
  }

  private GabeStates getGabeState() {
        if (gabeLeftClaw.get() && gabeRightClaw.get()) {
            if (gabeLeftIdentification.get() && gabeRightIndentification.get()) {
                  return GabeStates.CLOSED_WITH_BAR;
            } else if (!gabeLeftIdentification.get() && !gabeRightIndentification.get()) {
                  return GabeStates.CLOSED_WITHOUT_BAR;
            }
        } else if (!gabeLeftClaw.get() && !gabeRightClaw.get()) {
              return GabeStates.OPEN;
        }

        return GabeStates.UNKNOWN;
  }

  private RachelExtensionStates getExtensionState() {
        double currentVelocity = leftRachelFalcon.getSelectedSensorVelocity();
        double currentPosition = leftRachelFalcon.getSelectedSensorPosition();

        double extensionState = Math.signum(currentVelocity);

      // Check to see if the arms are moving and return just a movement direction
      if (Math.abs(currentVelocity) > 10.0) { 
            if (extensionState > 0) {
                  return RachelExtensionStates.EXTENDING;
            } else if (extensionState < 0) {
                  return RachelExtensionStates.RETRACTING;
            }
      }

      // Assume we are at a target position, because our velocity is small
      return RachelExtensionStates.findState(currentPosition);
  }

  public void stateMachine() {
      if(gabeLeftClaw.get() == true) {
            if(rachelLeftBarSensor.get() == true) {
                 if(leftRachelFalcon.get() == -1){ /*Retracting to Gabe position*/
                        //trust fall State
                        //Babys first Pull Up State
                        currentState = ClimberState.BABYS_FIRST_PULL_UP;
                 }
                 else /*leftReachelFalcon.get() == 0 or == 1*/ {
                       if (leftRachelFalcon.getSelectedSensorPosition() == Constants.GABE_HEIGHT_POSITION) {
                              //Sucessful Pull up State  - Rachel is at Gabe position
                              currentState = ClimberState.SUCESSFUL_PULL_UP;
                       }
                       else if (leftRachelFalcon.getSelectedSensorPosition() == Constants.REACH_CATCH_EXTENTION_HEIGHT)  {
                             currentState = ClimberState.TRUST_FALL;
                       }
                       else {
                             currentState = ClimberState.UNKNOWN;
                       }
                  
                 }
            }
            else /*gabeLeftIdentification.get() == false*/ {
                  if(leftRachelFalcon.getSensorCollection().isFwdLimitSwitchClosed() == 1) /*High Limit Switch triggered*/ {
                        //Babys First Reach State  - Fully extended position
                        currentState = ClimberState.BABYS_FIRST_REACH;
                  }
                  else /*leftRachelFalcon.getSensorCollection().isFwdLimitSwitchClosed() == 0 High Limit Switch not triggered*/ {
                        //Reaching State  
                        currentState = ClimberState.REACHING;
                  }
            }
      }
      
      else  /*(gabeLeftClaw.get() == false)*/  {       
            if(gabeLeftIdentification.get() == true) {
                  if(rachelLeftReach.get() == true) {
                        if(isRachelLeftMoving() == 1)  {
                              //Stegosaurus Reaching State
                              currentState = ClimberState.STEGOSAURUS_REACHING;
                        }
                        else /*leftRachelFalcon.getSensorCollection().isFwdLimitSwitchClosed() == 0) /*High Limit Switch not triggered*/ {
                             if(leftRachelFalcon.getSelectedSensorPosition() == Constants.FULLY_EXTENDED_HEIGHT)  /*Rachel is extending*/ {
                                    //Brontosaurus Reaching State   - ***At Fully Extended State***
                                    currentState = ClimberState.BRONTOSAURUS_REACHING;
                             }
                             else {
                                   //T Rex Reach Branch
                                   currentState = ClimberState.T_REX_REACH;
                             }
                        }
                  }
                  else /*rachelLeftReach.get() == false */ {
                        if(rachelLeftBarSensor.get() == true) {
                              if(isRachelLeftMoving() == 0) {
                                   if(leftRachelFalcon.getSelectedSensorPosition() == Constants.GABE_HEIGHT_POSITION) {
                                          //succesful hang state
                                          currentState = ClimberState.SUCESSFUL_HANG;
                                   }
                                   else if(leftRachelFalcon.getSelectedSensorPosition() == Constants.REACH_CATCH_EXTENTION_HEIGHT) {
                                         //Reach Caught State
                                         currentState = ClimberState.REACH_CAUGHT;
                                   }
                                   else {
                                         currentState = ClimberState.UNKNOWN;
                                   }
                              }
                              else /*(isRachelLeftMoving() == -1)*/ {
                                    //Reach Catch State
                                    currentState = ClimberState.REACH_CATCH;
                              }
                        }
                        else /*rachelLeftBarSensor.get() == false*/ {
                              if(leftRachelFalcon.getSelectedSensorPosition() == 0) {
                                    //Reach Pull State
                                    currentState = ClimberState.REACH_PULL;
                              }
                              else if(leftRachelFalcon.getSelectedSensorPosition() == 1) /*extending*/ {
                                    //Release Reach State
                                    currentState = ClimberState.RELEASE_REACH;
                              }
                              else /*leftRachelFalcon.getSelectedSensorPosition() == -1)*/ {
                                    //Reach Searching State    - Fully extended position
                                    currentState = ClimberState.REACH_SEARCHING;
                              }
                        }
                  }
            }
            else /*gabeLeftIdentification.get() == false*/ {
                 //Starting Config State  - fully retracted State
                 currentState = ClimberState.STARTING_CONFIG;
            }
      }
  }

  private int isRachelLeftMoving() {
      if(leftRachelFalcon.getMotorOutputPercent() == 0) {
            return 0;
      }
      else if(leftRachelFalcon.getMotorOutputPercent() > 0) {
            return 1;  //extending
      }
      else {
            return -1;  //retracting
      }
}

public ClimberState getCurrentState() {
      return currentState;
}
     

  @Override
  public void periodic() {
        stateMachine();
        SmartDashboard.putString("CurrentState", getCurrentState().name());
    // This method will be called once per scheduler run
  }
}

/*
Climber State Machine
  (  Drives back up past bar)
    Drives move forward till Climber forearm sensor trigger (Robot is in potion9) ("love potion number 9!")

  -Starting_Config-  Both arms upright, retracted, active bar sensor not triggered  (starting config)
        Gabe - Pneumatics Closed, Bar identification = No
        Rachel - Falcons Not moving, Pneumatics closed, Low climbing Limit Switch = yes, Active Bar sensor = no

  -Reaching-  Both arms upright, extending, active bar sensor not triggered, grabby open
        Gabe - Pneumatics Open, Bar identification = No
        Rachel - Falcons Moving(extending), Pneumatics closed, No limit switches triggered

  -Babys_First_Reach-  Both arms upright, extended, grabby open   (initial climbing position)
        Gabe - Pneumatics Open, Bar identification = No
        Rachel - Falcons Not moving, Pneumatics closed, Fully extended, Active Bar sensor = no

  -Babys_First_Pull_Up-  Both arms upright, retracting, active bar sensor triggered
        Gabe - Pneumatics Open, Bar identification = No
        Rachel - Falcons Moving(retracting to a position), Pneumatics closed, Active Bar Sensor = Yes

  -Sucessful_Pull_Up-  Both arms upright, retracted, Bar identification triggered
        Gabe - Pneumatics Open, Bar identification = no
        Rachel - Falcons Not moving, Pneumatics closed, Gabe height, Active Bar Sensor = Yes

  -Sucessful_Hang-  Both arms upright, retracted, grabby grabbed bar
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics closed, Gabe Height, Active Bar Sensor = Yes

  -Release_Reach-  Both arms upright, small extension, grabby grabbed bar (unhooking from previous bar)
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Moving(extending), Pneumatics closed, Active Bar Sensor = No 

  -T_Rex_Reach-  Gabe grabbed, small extension, active arm reaching
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not Moving, Pneumatics open, Active Bar Sensor = No

  -Stegosaurus_Reaching-  Gabe grabbed, extending, active arm reaching
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Moving(Extended), Pneumatics open, Active Bar Sensor = No

  -Brontosaurus_Reach-  Gabe grabbed, active arm reaching and extended
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics open, Fully extended, Active Bar Sensor = No

  -Reach_Pull-  Gabe grabbed, active arm upright and extended (active arm has been pulled into the bar in order to get ready to grab the next bar) 
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics closed, fully extended, Active Bar Sensor = No, 

  -Reach_Searching-  Gabe grabbed, active arm upright and retracting, active bar sensor not triggered
        Gabe - Pneumatics closed, Bar identification = yes
        Rachel - Falcons Moving(retracting), Pneumatics closed, Active Bar Sensor = No, 

  -Reach_Catch-  Gabe grabbed, active arm upright and retracting, active bar sensor triggered
        Gabe - Pneumatics closed, Bar Identification = yes
        Rachel - Falcons Moving(retracting) to a position, Pneumatics closed, Active Bar Sensor = yes

  -Reach_Caught- Gabe grabbed, active arm upright and retracting to a position, active bar sensor triggered
        Gabe - Pneumatics closed, Bar Identification = yes
        Rachel - Falcons not moving at position, Pneumatics closed, Active Bar Sensor = yes

  -Trust_Fall-  Gabe released, active arm upright and retracting, active bar sensor triggered
        Gabe - Pneumatics open, Bar Identification = No
        Rachel - Falcons not moving, Pneumatics closed, Active bar sensor = yes





All Motors and Sensors
Active Arm  - reachy arm - reachel (rachel)
  Active Bar sensor = Rachel sensor for holding the climbing bar
  2 Falcons to drive each arm
  2 Pneumatic Pistons to flop 
  # of Limit switches for extended/retracted states of climber in a box
  Limit switch for forearm limit switch (knowing when we are in contact with the bar both when on the ground climbing up and reaching to the next stage)


Gabe  - grabby arm   - gabe
  2 Pneumatic Pistons to grab bar
  2 Limit switches for bar identification 


*/