// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private PneumaticsControlModule pcm;
  private boolean intakeExtended = false;

  public Intake() {
    pcm = new PneumaticsControlModule(2);
    solenoid = pcm.makeSolenoid(Constants.INTAKE_SOLENOID_PORT);
    //solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_PORT);
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
    RawColor rawColor = colorSensor.getRawColor();
    double[] rawColorArray = new double[4];

    rawColorArray[0] = rawColor.red;
    rawColorArray[1] = rawColor.green;
    rawColorArray[2] = rawColor.blue;
    rawColorArray[3] = rawColor.ir;
    SmartDashboard.putNumberArray("sensorColor", rawColorArray);
  }

  public boolean checkColor(DriverStation.Alliance ourAlliance) {
    int greaterIndex = 1;
    int lessIndex = 4;

    switch (ourAlliance) {
      case Red:
        allianceColor = redTarget;
        greaterIndex = 0;
        lessIndex = 2;
        break;
      case Blue:
        allianceColor = blueTarget;
        greaterIndex = 2;
        lessIndex = 0;
        break;
      default:
        allianceColor = Color.kBlack;
    }
    RawColor rawColor = colorSensor.getRawColor();
    double[] rawColorArray = new double[4];

    rawColorArray[0] = rawColor.red;
    rawColorArray[1] = rawColor.green;
    rawColorArray[2] = rawColor.blue;
    rawColorArray[3] = rawColor.ir;
    //Color detectedColor = colorSensor.getColor();
    //ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());

    if (colorSensor.getProximity() >= Constants.DETECTABLE_DISTANCE) {
      if (rawColorArray[greaterIndex] > rawColorArray[lessIndex]) {
        return true;
      } else {
        return false;
      }
    }
    return true;

  }

  public void runIntake() {
    intakeMotor.set(Constants.INTAKE_SPEED);

  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void reverseIntake() {
    intakeMotor.set(-1.5 * Constants.INTAKE_SPEED);
  }

  public void extendIntake() {
    intakeExtended = true;
    solenoid.set(true);
  }

  public void retractIntake() {
    intakeExtended = false;
    solenoid.set(false);
  }

  public boolean isIntakeExtended() {
    return intakeExtended;
    
  }

  public static Intake getInstance() {
    if (intakeInstance == null) {
      intakeInstance = new Intake();
    }
    return intakeInstance;
  }
}
