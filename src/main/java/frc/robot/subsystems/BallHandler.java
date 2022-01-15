// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallHandler extends SubsystemBase {
  /** Creates a new BallHandler. */
  public BallHandler() {}
/** needs to extend(intake bar)
 * do we need to retract?
 * stop and start intake bar (spinning)
 * intake bar needs to spin in both directions (avoid jams)
 * start and stop lower wheels involved with intake/storage
 * need transit motors and control in both directions
 * Reverse for getting rid of ball?(what has to be reversed)
 * trap door for extake?
 * how many motors do we need at transit section?
 * How independent are the motors?
 * Retract intake bar during match or climbing?
 * 
 */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
