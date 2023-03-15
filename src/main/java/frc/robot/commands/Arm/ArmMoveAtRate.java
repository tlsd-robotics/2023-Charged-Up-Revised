// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveAtRate extends CommandBase {
  /** Creates a new ArmMoveAtRate. */
  Timer timer = new Timer();
  double rateDegreesPerSecond;
  ArmSubsystem arm;
  double intialAngle;
  public ArmMoveAtRate(double rateDegreesPerSecond, ArmSubsystem arm) {
    this.rateDegreesPerSecond = rateDegreesPerSecond;
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intialAngle = arm.getAngleSetpoint();
    timer.reset();
    timer.start();
    arm.enabled();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDegrees = intialAngle + (timer.get() * rateDegreesPerSecond);
    if (arm.getCurrentArmLength().AngleInValidRange(angleDegrees)) {
      arm.setAngle(angleDegrees);
      SmartDashboard.putBoolean("Angle Saftey Triggered: ", false);
    }
    else {
      SmartDashboard.putBoolean("Angle Saftey Triggered: ", true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
