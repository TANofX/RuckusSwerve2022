// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class JoystickUtils {
    public static double scaleDeadband(double input, double deadBand) {
        if (Math.abs(input) < deadBand) {
            return 0.0;
        }
        else 
            return Math.signum(input) * (Math.abs(input) - deadBand) / (1.0 - deadBand);
    }

    public static HatSwitchButton getHatSwitchButton(Joystick stick, HatSwitchButton.HatDirection direction) {
        return new HatSwitchButton(stick, direction);
    }
}
