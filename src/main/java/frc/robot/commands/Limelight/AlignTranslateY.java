// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignTranslateY extends CommandBase {
  PIDController pid = new PIDController(0.05, 0, 0);
  DrivetrainSubsystem drivetrain;
  Limelight limelight;
  boolean inverted;

  public AlignTranslateY(DrivetrainSubsystem drivetrain, Limelight limelight, boolean inverted) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.inverted = inverted;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(0,  pid.calculate((inverted ? -1 : 1) * limelight.getHorizontalError(), 0), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
