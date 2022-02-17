// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.commands.CloseHighGoalShoot;
import frc.commands.FarHighGoalShoot;
import frc.commands.LaunchpadGoalShoot;
import frc.commands.LowGoalShoot;
import frc.commands.ShootAll;
import frc.commands.ShootOne;
import frc.commands.StopShooter;
import frc.robot.subsystems.BallHandler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick joystick1 = new Joystick(Constants.JOYSTICK1_PORT);
  private JoystickButton stopShooterButton = new JoystickButton(joystick1, 5);
  private JoystickButton lowGoalShootButton = new JoystickButton(joystick1, 1);
  private JoystickButton closeHighGoalButton = new JoystickButton(joystick1, 2);
  private JoystickButton farHighGoalButton = new JoystickButton(joystick1, 3);
  private JoystickButton launchpadGoalShoot = new JoystickButton(joystick1, 4);
  private JoystickButton shootOne = new JoystickButton(joystick1, 6);
  private JoystickButton shootAll = new JoystickButton(joystick1, 7);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    BallHandler.getInstance();
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    stopShooterButton.whenPressed(new StopShooter());
    lowGoalShootButton.whenPressed(new LowGoalShoot());
    closeHighGoalButton.whenPressed(new CloseHighGoalShoot());
    farHighGoalButton.whenPressed(new FarHighGoalShoot());
    launchpadGoalShoot.whenPressed(new LaunchpadGoalShoot());
    shootOne.whenPressed(new ShootOne());
    shootAll.whenPressed(new ShootAll());

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
}
