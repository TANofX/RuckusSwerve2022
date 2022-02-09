// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private Spark intakeMotor;
  private ColorSensorV3 colorSensor;
  private Solenoid solenoid;
  private static Intake intakeInstance;
  
  

  public Intake() {
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.INTAKE_SOLENOID_PORT);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    ColorMatch colorMatcher = new ColorMatch();
    Color BlueTarget = ColorMatch.makeColor(0.169, 0.405, 0.426);
    Color RedTarget = ColorMatch.makeColor(0.528, 0.347, 0.126);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    colorSensor.getProximity();
    
  }
  public void runIntake() {
    solenoid.set(true);
    intakeMotor.set(0.5);
    if (colorSensor.getProximity() >= 300 ) {
      colorSensor.getColor();

    }
    ColorMatchResult match = colorMatcher.matchClosestColor(detectColor)
    
  }

  public void stopIntake() {
    solenoid.set(false);
    intakeMotor.stopMotor();
  }

  public static Intake getInstance() {
    if (intakeInstance == null) {
      intakeInstance = new Intake();
    }
    return intakeInstance;
  }
}
