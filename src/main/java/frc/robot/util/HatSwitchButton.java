// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

/** Add your docs here. */
public class HatSwitchButton extends Button {
    public enum HatDirection {
        UP,
        DOWN,
        LEFT,
        RIGHT
    }

    private Joystick flightStick;
    private int axis;
    private int value;

    protected HatSwitchButton(Joystick stick, HatDirection direction) {
        flightStick = stick;
        switch (direction) {
            case UP:
                axis = 5;
                value = -1;
                break;
            case DOWN:
                axis = 5;
                value = 1;
                break;
            case LEFT:
                axis = 6;
                value = -1;
                break;
            case RIGHT:
                axis = 6;
                value = 1;
                break;
        }
    }

    @Override
    public boolean get() {
        return flightStick.getRawAxis(axis) == value;
    }
    
}
