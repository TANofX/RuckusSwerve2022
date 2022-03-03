// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public enum ShooterSpeeds {
LOWGOAL(7000),
CLOSEHIGHGOAL(12000),
FARHIGHGOAL(13000),
LAUNCHPADGOAL(13500),
OFF(0);

private double actualSpeed;

    private ShooterSpeeds(double Speed) {
        
        actualSpeed = Speed;

    }

    public double getMotorSpeed() {
        return actualSpeed;
    }
}
