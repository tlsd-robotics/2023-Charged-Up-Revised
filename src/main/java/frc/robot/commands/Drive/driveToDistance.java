// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.navX;
import frc.robot.commands.navX.zeroNavxDisplacement;
import frc.robot.subsystems.DrivetrainSubsystem;

public class driveToDistance extends CommandBase {
  DrivetrainSubsystem drivetrain;
  Double speed;
  double distX;
  double tolerance = .1;
  /** Creates a new driveToDistance. */
  public driveToDistance(DrivetrainSubsystem Drivetrain, double distX, double speed) {
    this.speed = speed;
    this.distX = distX;
    addRequirements(Drivetrain);
    drivetrain = Drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new zeroNavxDisplacement();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(speed, 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(navX.navX.getDisplacementX()) >= distX - tolerance && Math.abs(navX.navX.getDisplacementX()) <= distX + tolerance;
  }
}
