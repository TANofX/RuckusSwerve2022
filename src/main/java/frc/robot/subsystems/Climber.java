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
  (  Drives back up past bar)
    Drives move forward till Climber forearm sensor trigger (Robot is in potion9) ("love potion number 9!")

  -Starting_Config-  Both arms upright, retracted, active bar sensor not triggered  (starting config)
        Gabe - Pneumatics Closed, Bar identification = No
        Rachel - Falcons Not moving, Pneumatics closed, Low climbing Limit Switch = yes, Active Bar sensor = no

  -Reaching-  Both arms upright, extending, active bar sensor not triggered, grabby open
        Gabe - Pneumatics Open, Bar identification = No
        Rachel - Falcons Moving(direction?), Pneumatics closed, No limit switches triggered

  -Babys_First_Reach-  Both arms upright, extended, grabby open   (initial climbing position)
        Gabe - Pneumatics Open, Bar identification = No
        Rachel - Falcons Not moving, Pneumatics closed, Up climbing Limit Switch = yes, Active Bar sensor = no

  -Babys_First_Pull_Up-  Both arms upright, retracting, active bar sensor triggered
        Gabe - Pneumatics Open, Bar identification = No
        Rachel - Falcons Moving(direction?), Pneumatics closed, Active Bar Sensor = Yes

  -Sucessful_Pull_Up-  Both arms upright, retracted, Bar identification triggered
        Gabe - Pneumatics Open, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics closed, Low climbing Limit Switch = yes, Active Bar Sensor = Yes

  -Sucessful_Hang-  Both arms upright, retracted, grabby grabbed bar
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics closed, Low climbing Limit Switch = Yes, Active Bar Sensor = Yes

  -Release_Reach-  Both arms upright, small extension, grabby grabbed bar (unhooking from previous bar)
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Moving(direction?), Pneumatics closed, Low climbing Limit Switch = no, Active Bar Sensor = No 

  -T_Rex_Reach-  Gabe grabbed, small extension, active arm reaching
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not Moving, Pneumatics open, Low climbing limit switch = no, Active Bar Sensor = No

  -Stegosaurus_Reaching-  Gabe grabbed, extending, active arm reaching
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Moving(direction?), Pneumatics open, Active Bar Sensor = No

  -Brontosaurus_Reach-  Gabe grabbed, active arm reaching and extended
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics open, Up climbing limit switch = yes, Active Bar Sensor = No

  -Reach_Pull-  Gabe grabbed, active arm upright and extended (active arm has been pulled into the bar in order to get ready to grab the next bar) 
        Gabe - Pneumatics Closed, Bar identification = yes
        Rachel - Falcons Not moving, Pneumatics closed, Up climbing limit switch = yes, Active Bar Sensor = No, 

  -Reach_Searching-  Gabe grabbed, active arm upright and retracting, active bar sensor not triggered
        Gabe - Pneumatics closed, Bar identification = yes
        Rachel - Falcons Moving(direction?), Pneumatics closed, Active Bar Sensor = No, 

  -Reach_Catch-  Gabe grabbed, active arm upright and retracting, active bar sensor triggered
        Gabe - Pneumatics closed, Bar Identification = yes
        Rachel - Falcons Moving(direction?), Pneumatics closed, Active Bar Sensor = yes

  -Reach_Caught- Gabe grabbed, active arm upright and retracting to a position, active bar sensor triggered
        Gabe - Pneumatics closed, Bar Identification = yes
        Rachel - Falcons Moving(direction?) to a position, Pneumatics closed, Active Bar Sensor = yes

  -Trust_Fall-  Gabe released, active arm upright and retracting, active bar sensor triggered
        Gabe - Pneumatics open, Bar Identification = No
        Rachel - Falcons Moving(direction?), Pneumatics closed, Active bar sensor = yes





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