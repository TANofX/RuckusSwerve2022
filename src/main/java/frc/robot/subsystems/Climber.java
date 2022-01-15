// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/*
Climber State Machine
    Drives back up past bar
    Drives move forward till Climber long senscers trigger (Robot is in potion9) ("love potion number 9!")
    Both arms upright, retracted  (starting config)
    Both arms upright, extended, grabby open   (initial climbing position)
    Both arms upright, retracted   ()
    Both arms upright, retracted, grabby grabbed bar
    Both arms upright, small extension, grabby grabbed bar (unhooking from previous bar)
    Passive arm grabbed, small extension, active arm reaching
    Passive arm grabbed, active arm reaching and extended
    Passive arm grabbed, active arm upright and extended (active arm has been pulled into the bar in order to get ready to grab the next bar) (Need to worry about the bending force on arms?)
    Passive arm grabbed, active arm upright and retracting, active bar sensor not triggered
    Passive arm grabbed, active arm upright and retracting, active bar sensor triggered
    Passive arm released, active arm upright and retracting, active bar sensor triggered
    Both arms upright, retracted ()

All Motors and Sensors
  Active Arm  - reachy arm - reachel (rachel)
    2 Falcons to drive each arm
    2 Pneumatic Pistons to flop 
    # of Limit switches for extended/retracted states of climber in a box
    Limit switch for forearm limit switch (knowing when we are in contact with the bar both when on the ground climbing up and reaching to the next stage)

  Passive Arm  - grabby arm  - gabe
    2 Pneumatic Pistons to grab bar
    2 Limit switches for bar identification 


*/