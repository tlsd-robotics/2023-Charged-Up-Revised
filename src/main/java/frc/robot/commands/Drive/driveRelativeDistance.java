// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveRelativeDistance extends CommandBase {
  DrivetrainSubsystem drivetrain;

  private final double toleranceDistance = .1;
  private final double toleranceAngle = 1;

  private double valX;
  private double valY;
  private double valZ;

  private double initialX;
  private double initialY;
  private double initialZ;

  PIDController pidX = new PIDController(1.0, 0.0, 0.0);
  PIDController pidY = new PIDController(1.0, 0.0, 0.0);
  PIDController pidZ = new PIDController(1.0, 0.0, 0.0);

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

    pidX.setTolerance(toleranceDistance);
    pidY.setTolerance(toleranceDistance);
    pidZ.setTolerance(toleranceAngle);

    initialX = drivetrain.getPose2d().getX();
    initialY = drivetrain.getPose2d().getY();
    initialZ = drivetrain.getRotation().getDegrees();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.drive(new ChassisSpeeds(
      pidX.calculate(drivetrain.getPose2d().getX(), initialX + valX), 
      pidY.calculate(drivetrain.getPose2d().getY(), initialY + valY), 
      pidZ.calculate(drivetrain.getRotation().getDegrees(), initialZ + valZ)
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
