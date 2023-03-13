// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.UtilityClasses.AxisSupplier;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AllignToTarget extends CommandBase {
  AxisSupplier rightZAxis = new AxisSupplier(RobotContainer.rightJoy, AxisType.kZ.value, true, 0.05, true);

  DrivetrainSubsystem drivetrain;
  Limelight limelight;

  private final double DRIVE_TOLERANCE = 0.25;
  private final double STEER_TOLERANCE = 0.25;

  // =========== Construct PID Controllers =============
  PIDController pidX = new PIDController(0.5, 0, 0);
  PIDController pidY = new PIDController(0.5, 0, 0);
  PIDController pidZ = new PIDController(-0.01, 0, 0);
  
  public AllignToTarget(DrivetrainSubsystem Drivetrain, Limelight Limelight) {
    this.drivetrain = Drivetrain;
    this.limelight = Limelight;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX.setTolerance(DRIVE_TOLERANCE);
    pidY.setTolerance(DRIVE_TOLERANCE);
    pidZ.setTolerance(STEER_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (limelight.getIsTargetFound()) {
      if (limelight.getPipelineInt() == Limelight.aprilTag.id) {

        drivetrain.drive(new ChassisSpeeds(
          pidY.calculate(limelight.getVerticalError(), 0),
          pidX.calculate(limelight.getHorizontalError(), 0),
          pidZ.calculate(limelight.getTagYaw(), 0)));

      } else {

        drivetrain.drive(new ChassisSpeeds(
          pidY.calculate(limelight.getVerticalError(), 0),
          pidX.calculate(limelight.getHorizontalError(), 0),
          rightZAxis.getAsDouble()));

      }
    } else {
      drivetrain.drive(new ChassisSpeeds(0, 0, rightZAxis.getAsDouble()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidX.atSetpoint() && pidY.atSetpoint() && pidZ.atSetpoint();
  }
}
