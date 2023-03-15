// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmLength;

public class ArmToLength extends CommandBase {
  /** Creates a new ArmToLength. */

  private final int TRANSITION_TIME_SECONDS = 1;

  ArmLength length;
  ArmSubsystem arm;
  Timer timer = new Timer();
  public ArmToLength(ArmLength length, ArmSubsystem arm) {
    this.length = length;
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.

  //TODO: Add safety checks, pressure based timing
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if (length.AngleInValidRange(arm.getAngleSetpoint()) && length.AngleInValidRange(arm.getEncoderAngle())) {
      arm.setArmLength(length);
      SmartDashboard.putBoolean("Length Saftey Triggered: ", false);
    }
    else {
      SmartDashboard.putBoolean("Length Saftey Triggered: ", true);
    }
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
    return timer.get() >= TRANSITION_TIME_SECONDS;
  }
}
