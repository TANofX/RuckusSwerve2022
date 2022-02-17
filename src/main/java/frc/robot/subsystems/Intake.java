// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor;
  private ColorSensorV3 colorSensor;
  private Solenoid solenoid;
  private static Intake intakeInstance;
  private ColorMatch colorMatcher = new ColorMatch();
  private Color blueTarget = new Color(0.169, 0.405, 0.426);
  private Color redTarget = new Color(0.528, 0.347, 0.126);
  private Color allianceColor;

  public Intake() {
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_PORT);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    colorMatcher.addColorMatch(blueTarget);
    colorMatcher.addColorMatch(redTarget);
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(29, 10);
    intakeMotor.setOpenLoopRampRate(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public boolean checkColor(DriverStation.Alliance ourAlliance) {
    switch (ourAlliance) {
      case Red:
        allianceColor = redTarget;
        break;
      case Blue:
        allianceColor = blueTarget;
        break;
      default:
        allianceColor = Color.kBlack;
    }

    if (colorSensor.getProximity() >= Constants.DETECTABLE_DISTANCE) {
      Color detectedColor = colorSensor.getColor();
      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
      if (match.color == allianceColor) {
        return true;
      } else {
        return false;
      }
    }
    return true;

  }

  public void runIntake() {
    intakeMotor.set(0.5);

  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void reverseIntake() {
    intakeMotor.set(-0.5);
  }

  public void extendIntake() {
    solenoid.set(true);
  }

  public void retractIntake() {
    solenoid.set(false);
  }

  public static Intake getInstance() {
    if (intakeInstance == null) {
      intakeInstance = new Intake();
    }
    return intakeInstance;
  }
}
