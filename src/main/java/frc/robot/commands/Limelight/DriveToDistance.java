// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToDistance extends CommandBase {

  PIDController pid = new PIDController(0.1, 0, 0);

  DrivetrainSubsystem drivetrain;
  Limelight limelight;
  double distanceMeters;
  double targetHeightMeters;
  double maxVelocityMetersPerSecond;
  boolean positiveX;
  //Drives a specified distance from a target of specified height
  //Works with a limelight mounted on the front or rear of robot (Specify direction with last parameter)
  public DriveToDistance(DrivetrainSubsystem drivetrain, Limelight limelight, double distanceMeters, double maxVelocityMetersPerSecond, double targetHeightMeters, boolean positiveX) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.distanceMeters = distanceMeters;
    this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
    this.targetHeightMeters = targetHeightMeters;
    this.positiveX = positiveX;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds((positiveX ? 1 : -1) * pid.calculate(limelight.getDistanceToTarget(targetHeightMeters), distanceMeters), 
    0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
