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
  public enum ArmLength {
    RETRACTED,
    HALF_EXTENDED,
    FULLY_EXTENDED
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
  }

  public double getEncoderAngle() {
    return encoder.get() - ENCODER_OFFSET;
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
      case HALF_EXTENDED: {
        lowerCylinders.set(Value.kForward);
        upperCylinders.set(Value.kReverse);
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
      double motorSpeed = pid.calculate(encoder.get() - ENCODER_OFFSET, targetAngle) + feedForward.calculate(Math.toRadians(targetAngle), 1);
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
