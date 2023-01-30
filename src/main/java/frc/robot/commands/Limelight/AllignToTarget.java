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

  // =========== Construct PID Controllers =============
  PIDController pidX = new PIDController(0.5, 0, 0);
  PIDController pidY = new PIDController(0.5, 0, 0);
  
  public AllignToTarget(DrivetrainSubsystem Drivetrain) {
    this.drivetrain = Drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX.setTolerance(0.25);
    pidY.setTolerance(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Limelight.limelight1.getIsTargetFound()) {
      drivetrain.drive(new ChassisSpeeds(
          pidY.calculate(Limelight.limelight1.getVerticalError(), 0),
          pidX.calculate(Limelight.limelight1.getHorizontalError(), 0),
          rightZAxis.getAsDouble()));
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
    return false;
  }
}
