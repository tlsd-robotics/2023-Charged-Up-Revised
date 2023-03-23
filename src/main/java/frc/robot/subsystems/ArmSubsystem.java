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
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityClasses.Util;
import frc.robot.subsystems.ArmSubsystem.ArmLength;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax right = new CANSparkMax(29, MotorType.kBrushless);
  private CANSparkMax left = new CANSparkMax(23, MotorType.kBrushless);
  private MotorControllerGroup angleMotors = new MotorControllerGroup(left, right);

  private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  final double ENCODER_OFFSET = 55.85;

  private ArmFeedforward feedForward = new ArmFeedforward(0, 0, 0, 0);
  private PIDController pid = new PIDController(.03, 0, 0.0005);

  private DigitalInput armLimitSwitchFront = new DigitalInput(1);
  private DigitalInput armLimitSwitchRear = new DigitalInput(2);

  private double targetAngle;
  private boolean angleControlEnabled = false;

  DoubleSolenoid lowerCylinders = new DoubleSolenoid(3, PneumaticsModuleType.REVPH, 1, 14);
  DoubleSolenoid upperCylinders = new DoubleSolenoid(3, PneumaticsModuleType.REVPH, 2, 13);

  public enum ArmSetpoint {

    RETRACTED(-48.4, ArmLength.RETRACTED),
    FRONT_FLOOR(-36.5, ArmLength.FULLY_EXTENDED),
    REAR_FLOOR(164, ArmLength.UPPER_EXTENDED),
    MID_CUBE(9.6, ArmLength.FULLY_EXTENDED),
    MID_CONE(15.1, ArmLength.FULLY_EXTENDED),
    UPPER_CUBE(158, ArmLength.FULLY_EXTENDED),
    UPPER_CONE(147, ArmLength.FULLY_EXTENDED),
    SUBSTATION(16, ArmLength.UPPER_EXTENDED);
  
    public final double angleDegrees;
    public final ArmLength length;
  
    ArmSetpoint(double angleDegrees, ArmLength length) {
      this.angleDegrees = angleDegrees;
      this.length = length;
    }
  }

  public enum ArmLength {
    RETRACTED(new double[][] {{-48.4, 241.2}}),
    LOWER_EXTENDED(new double[][] {{-44.25, 200}}),
    UPPER_EXTENDED(new double[][] {{-44.6, 220}}),
    FULLY_EXTENDED(new double[][] {{-41, 190}});

    private static final ArmLength[] vals = ArmLength.values();

    private final double[][] VALID_RANGES;

    ArmLength(double[][] validRangles) {
      this.VALID_RANGES = validRangles;
    }

    public boolean AngleInValidRange(double angleDegrees) {
      for (int i = 0; i < this.VALID_RANGES.length; i++) {
        if (Util.inRange(angleDegrees, this.VALID_RANGES[i][0], this.VALID_RANGES[i][1])) {
          return true;
        }
      }
      return false;
    }

    public ArmLength next() {
      return vals[(this.ordinal() < vals.length - 1) ? (this.ordinal() + 1) : (this.ordinal())];
    }

    public ArmLength previous() {
      return vals[(this.ordinal() != 0) ? (this.ordinal() - 1) : (this.ordinal())];
    }
  };
  ArmLength currentArmLength = ArmLength.RETRACTED;

  /** Creates a new ShoulderSubsystem. */
  public ArmSubsystem() {
    right.restoreFactoryDefaults();
    left.restoreFactoryDefaults();
    right.setInverted(true);
    left.setInverted(false);
    right.burnFlash();
    left.burnFlash();
    encoder.setDistancePerRotation(360);
    upperCylinders.set(Value.kReverse);
    lowerCylinders.set(Value.kReverse);
    targetAngle = getEncoderAngle();
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
        angleMotors.set(motorSpeed);
    }
    else {
      angleMotors.set(0);
    }

    SmartDashboard.putNumber("Current Angle: ", getEncoderAngle());
    SmartDashboard.putNumber("Current Setpoint: ", getAngleSetpoint());
    SmartDashboard.putNumber("Current Length Index: ", currentArmLength.ordinal());
    SmartDashboard.putBoolean("Arm Enabled: ", angleControlEnabled);
  }
}


