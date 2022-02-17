// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int SHOOTER_CURRENT_LIMIT = 39;
    public static final int SHOOTER_THRESHOLD_CURRENT = 60;
    public static final double SHOOTER_THRESHOLD_TIMEOUT = 0.01;
    public static final double SHOOTER_F = 0.05;
    public static final double SHOOTER_P = 0.5;
    public static final double SHOOTER_I = 0.6;
    public static final double SHOOTER_SPEED = 0.9;
    public static final double SHOOTER_SPIN_ERROR = 0.2;
    public static final int JOYSTICK1_PORT = 0;
    public static final int HANDLERSENSOR_1_PORT = 1;
    public static final int HANDLERSENSOR_2_PORT = 2;
    public static final int HANDLERSENSOR_3_PORT = 3;
    public static final int HANDLERSENSOR_4_PORT = 4;
    public static final int INTAKE_SOLENOID_PORT = 0;
    public static final int DETECTABLE_DISTANCE = 500;
    public static final double REVERSE_INTAKE_TIMEOUT = 0.5;
    public static final int PRIMARY_SHOOTER_ID = 10;
    public static final int SECONDARY_SHOOTER_ID = 11;
    public static final int TRANSIT_MOTOR_ID = 8;
    public static final int INTAKE_MOTOR_ID = 9;
    public static final double TRANSIT_MOTOR_SPEED = 0.3;
}
