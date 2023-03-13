// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityClasses.Util;
import frc.robot.subsystems.ArmSubsystem.ArmLength;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax right = new CANSparkMax(29, MotorType.kBrushless);
  private CANSparkMax left = new CANSparkMax(23, MotorType.kBrushless);
  private MotorControllerGroup angleMotors = new MotorControllerGroup(left, right);

  private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  final double ENCODER_OFFSET = 62.3;

  private ArmFeedforward feedForward = new ArmFeedforward(0, 0, 0, 0);
  private PIDController pid = new PIDController(.1, 0, 0);

  private DigitalInput armLimitSwitchFront = new DigitalInput(1);
  private DigitalInput armLimitSwitchRear = new DigitalInput(2);

  private double targetAngle = 0;
  private boolean angleControlEnabled = false;

  DoubleSolenoid lowerCylinders = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  DoubleSolenoid upperCylinders = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);

  public enum ArmSetpoint {

    RETRACTED(-60, ArmLength.RETRACTED),
    FRONT_FLOOR(-50, ArmLength.RETRACTED),
    REAR_FLOOR(70, ArmLength.RETRACTED),
    MID_CUBE(180, ArmLength.RETRACTED),
    MID_CONE(175, ArmLength.RETRACTED),
    UPPER_CUBE(170, ArmLength.RETRACTED),
    UPPER_CONE(165, ArmLength.RETRACTED),
    SUBSTATION(170, ArmLength.RETRACTED);
  
    public final double angleDegrees;
    public final ArmLength length;
  
    ArmSetpoint(double angleDegrees, ArmLength length) {
      this.angleDegrees = angleDegrees;
      this.length = length;
    }
  }

  public enum ArmLength {
    RETRACTED,
    LOWER_EXTENDED,
    UPPER_EXTENDED,
    FULLY_EXTENDED;

    ArmLength[] vals = ArmLength.values();

    public ArmLength next() {
      return vals[(this.ordinal() + 1 != vals.length) ? (this.ordinal() + 1) : (this.ordinal())];
    }

    public ArmLength previous() {
      return vals[(this.ordinal() != 0) ? ( this.ordinal() - 1) : (this.ordinal())];
    }
  };
  ArmLength currentArmLength;

  /** Creates a new ShoulderSubsystem. */
  public ArmSubsystem() {
    right.restoreFactoryDefaults();
    left.restoreFactoryDefaults();
    right.setInverted(true);
    left.setInverted(false);
    right.burnFlash();
    left.burnFlash();
    encoder.setDistancePerRotation(360);
  }

  public double getEncoderAngle() {
    return encoder.getDistance() - ENCODER_OFFSET;
  }

  public double getAngleSetpoint() {
    return targetAngle;
  }

  public void enabled() {
    angleControlEnabled = true;
  }
  
  public void disabled() {
    angleControlEnabled = false;
  }

  public void setAngle(double angle) {

    targetAngle = angle;
  }

  //TODO: Consider adding safety checks for extension.
  public void setArmLength(ArmLength length) {
    switch (length) {
      case RETRACTED: {
        lowerCylinders.set(Value.kReverse);
        upperCylinders.set(Value.kReverse);
        break;
      }
      case LOWER_EXTENDED: {
        lowerCylinders.set(Value.kForward);
        upperCylinders.set(Value.kReverse);
        break;
      }
      case UPPER_EXTENDED: {
        lowerCylinders.set(Value.kReverse);
        upperCylinders.set(Value.kForward);
        break;
      }
      case FULLY_EXTENDED: {
        lowerCylinders.set(Value.kForward);
        upperCylinders.set(Value.kForward);
        break;
      }
    }
    currentArmLength = length;
  }

  public ArmLength getCurrentArmLength() {
    return currentArmLength;
  }

  @Override
  public void periodic() {
    if (angleControlEnabled) {
      double motorSpeed = pid.calculate(getEncoderAngle(), targetAngle) + feedForward.calculate(Math.toRadians(targetAngle), 1);
      if (armLimitSwitchFront.get() && (motorSpeed < 0)) {
        angleMotors.set(0);
        return;
      }
      else if (armLimitSwitchRear.get() && (motorSpeed > 0)) {
        angleMotors.set(0);
        return;
      }
      else {
        angleMotors.set(motorSpeed);
      }
    }
    else {
      angleMotors.set(0);
    }
  }
}


