// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmLength;

public class ArmToLength extends CommandBase {
  /** Creates a new ArmToLength. */
  ArmLength length;
  ArmSubsystem arm;
  public ArmToLength(ArmLength length, ArmSubsystem arm) {
    this.length = length;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.

  //TODO: Add safety checks, pressure based timing
  @Override
  public void initialize() {
    arm.setArmLength(length);
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
    return true;
  }
}
