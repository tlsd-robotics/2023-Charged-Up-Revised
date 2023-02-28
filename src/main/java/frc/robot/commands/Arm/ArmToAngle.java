// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.UtilityClasses.Util;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmLength;

public class ArmToAngle extends CommandBase {
  /** Creates a new ArmToSetpoint. */
  double angleDegrees;
  ArmLength length;
  ArmSubsystem arm;
  final double ANGLE_TOLERANCE = 1;
  final int STABLE_INTERATIONS = 15;
  int inRangeIterations = 0;
  public ArmToAngle(double angleDegrees, ArmLength length, ArmSubsystem arm) {
    this.angleDegrees = angleDegrees;
    this.length = length;
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.enabled();
    arm.setAngle(angleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //The arm must be within the angle range for STABLE_ITERATIONS consecutively before its position is considered stable, and the command finishes
    if (Util.inRange(arm.getEncoderAngle(), angleDegrees - ANGLE_TOLERANCE, angleDegrees + ANGLE_TOLERANCE)) {
      inRangeIterations ++;
      if (inRangeIterations >= STABLE_INTERATIONS) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      inRangeIterations = 0;
      return false;
    }
  }
}
