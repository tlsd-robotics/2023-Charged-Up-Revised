// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveRelativeDistance extends CommandBase {
  DrivetrainSubsystem drivetrain;

  private final double TOLERANCE_DISTANCE = .1;
  private final double TOLERANCE_ANGLE = 1;

  private double valX;
  private double valY;
  private double valZ;

  private double initialX;
  private double initialY;
  private double initialZ;

  PIDController pidX = new PIDController(1.5, 0.0, 0.0);
  PIDController pidY = new PIDController(1.5, 0.0, 0.0);
  PIDController pidZ = new PIDController(1.5, 0.0, 0.0);

  public DriveRelativeDistance(DrivetrainSubsystem Drivetrain, double ValX, double ValY, double ValZ) {
    this.drivetrain = Drivetrain;
    this.valX = ValX;
    this.valY = ValY;
    this.valZ = ValZ;
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidX.setTolerance(TOLERANCE_DISTANCE);
    pidY.setTolerance(TOLERANCE_DISTANCE);
    pidZ.setTolerance(TOLERANCE_ANGLE);

    initialX = drivetrain.getPose2d().getX();
    initialY = drivetrain.getPose2d().getY();
    initialZ = drivetrain.getOdometryRotation().getDegrees();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.drive(new ChassisSpeeds(
      MathUtil.clamp(pidX.calculate(drivetrain.getPose2d().getX(), initialX + valX), -DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND), 
      MathUtil.clamp(pidY.calculate(drivetrain.getPose2d().getY(), initialY + valY), -DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND), 
      MathUtil.clamp(pidZ.calculate(drivetrain.getOdometryRotation().getDegrees(), initialZ + valZ), -DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidX.atSetpoint() && pidY.atSetpoint() && pidZ.atSetpoint();
  }
}
