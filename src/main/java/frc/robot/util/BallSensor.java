// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class BallSensor extends AnalogInput {

    private double triggerVoltage = 0.9;

    public boolean isTriggered () {
        return getVoltage() >= triggerVoltage;
    }
    public BallSensor(int channel) {
        super(channel);
        //TODO Auto-generated constructor stub
    }
    public BallSensor(int channel, double voltage) {
        this (channel);
        triggerVoltage = voltage;
    }
}
