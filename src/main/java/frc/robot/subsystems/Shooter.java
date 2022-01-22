// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new WPI_TalonFX(0);
    //to do: get correct can ID
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT, Constants.SHOOTER_THRESHOLD_CURRENT, Constants.SHOOTER_THRESHOLD_TIMEOUT));

    shooterMotor.selectProfileSlot(0,0);

    shooterMotor.config_kF(0, Constants.SHOOTER_F, 0);
    shooterMotor.config_kP(0, Constants.SHOOTER_P, 0);
    shooterMotor.config_kI(0, Constants.SHOOTER_I, 0);

    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);

    shooterMotor.setSelectedSensorPosition(0);

  }
/** change angles of shooter?
 * Does shooter wheels have the same speed or is it different?
 * Is the shooter folded in like the intake is? (will it interfere with climber)
 * How do we get ball up the shooter?
 * Falcon motor for shooter

 */

private WPI_TalonFX shooterMotor;
private ShooterSpeeds targetShooterSpeeds = ShooterSpeeds.OFF;





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }

  public void startShooter(ShooterSpeeds SpeedToShoot) {
    shooterMotor.set(ControlMode.Velocity, SpeedToShoot.getMotorSpeed());
    targetShooterSpeeds = SpeedToShoot;

    
  }

  public void stopShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 0);

  }

  public double getShooterSpeed() {
    return shooterMotor.getSelectedSensorVelocity(0);
  }

  public boolean correctSpeed() {
  
   if(Math.abs(targetShooterSpeeds.getMotorSpeed() - getShooterSpeed()) < Constants.SHOOTER_SPIN_ERROR) {
     return true;
   }
  return false; 

  }
}
