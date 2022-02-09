// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import commands.MoveRachelWithJoystick;
import commands.SetClimberState;
import commands.ToggleGabeClaw;
import commands.ToggleRachelReach;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final Joystick m_stick = new Joystick(Constants.STICK);
  final Joystick m_xbox = new Joystick(Constants.XBOX);

  final JoystickButton toggleGabe = new JoystickButton(m_stick, Constants.GABE_PNEUMATICS_BUTTON);
  final JoystickButton toggleRachel = new JoystickButton(m_stick, Constants.RACHEL_REACH_PNEUMATICS_BUTTON);
  final JoystickButton highClimb = new JoystickButton(m_stick, Constants.CLIMB_HIGH_BAR);
  final JoystickButton traversalClimb = new JoystickButton(m_stick, Constants.CLIMB_TRAVERSAL_BAR);

  final JoystickButton lockRachelMoveJoystick = new JoystickButton(m_stick, Constants.LOCK_RACHEL_MOVE_JOYSTICK);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Climber.getInstance();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleGabe.whenPressed(new ToggleGabeClaw());
    toggleRachel.whenPressed(new ToggleRachelReach());

    highClimb.whenPressed(getHighClimbCommand());
    traversalClimb.whenPressed(getTraversalClimbCommand());

    lockRachelMoveJoystick.whileHeld(new MoveRachelWithJoystick(Climber.getInstance(), m_stick));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  private Command getHighClimbCommand() {
    return new SequentialCommandGroup(
        new SetClimberState(ClimberState.BABYS_FIRST_REACH),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL));
  }

  private Command getTraversalClimbCommand() {
    return new SequentialCommandGroup(
        new SetClimberState(ClimberState.BABYS_FIRST_REACH),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL));
  }
}
