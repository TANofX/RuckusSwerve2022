// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
                        ///////////////////////  D R I V I N G ///////////////////////
    //BUTTON INPUTS FOR FLIGHT STICK CONTROLLER
    public static int STICK = 1;

    public static int GABE_PNEUMATICS_BUTTON = 1;
    public static int RACHEL_REACH_PNEUMATICS_BUTTON = 2;

 //   public static  int CLIMB_TRAVERSAL_BAR = 10;
 //   public static  int CLIMB_HIGH_BAR = 8;

  //  public static  int CANCEL_CLIMBER = 10;
    public static  int CALIBRATE_CLIMBER = 12;

    public static int LOCK_RACHEL_MOVE_JOYSTICK = 4;
    public static int MOVE_RACHEL_AXIS = 0;

    //BUTTON INPUTS FOR XBOX CONTROLLER
    public static final int XBOX = 0;

                        ///////////////////////  C L I M B E R  ///////////////////////
    //  CLIMBER SUBSYSTEM CONSTANTS
    public static int RACHEL_UP_LEFT_LIMIT = 0;
    public static int RACHEL_UP_RIGHT_LIMIT = 1;
    public static int RACHEL_DOWN_LEFT_LIMIT = 2;
    public static int RACHEL_DOWN_RIGHT_LIMIT = 3;

    public static int GABE_CLAW = 1;
    public static int RACHEL_FORWARD_REACH = 3;
    public static int RACHEL_BACKWARD_REACH = 2;

    public static int GABE_LEFT_IDENTIFICATION = 3;
    public static int GABE_RIGHT_IDENTIFICATION = 2;
    public static int RACHEL_LEFT_BAR_SENSOR = 0;
    public static int RACHEL_RIGHT_BAR_SENSOR = 1;

    public static int RIGHT_RACHEL_FALCON = 12;
    public static int LEFT_RACHEL_FALCON = 13;

    public static double RACHEL_F = 0;
    public static double RACHEL_I = 0;
    public static double RACHEL_P = 0;

    public static double CLIMBER_THRESHOLD_CURRENT = 60;
    public static double CLIMBER_CURRENT_LIMIT = 39;
    public static double CLIMBER_THRESHOLD_TIMEOUT = 0.1;


    //Constants for Climber Heights 
    public static int FULLY_EXTENDED_HEIGHT = 10;
    public static int GABE_HEIGHT_POSITION = 5;
    public static int REACH_CATCH_EXTENTION_HEIGHT = 2;

    public static int CLIMB_HEIGHT_THRESHOLD = 1;
    public static double CLIMBER_MAX_VELOCITY = 500;




}
