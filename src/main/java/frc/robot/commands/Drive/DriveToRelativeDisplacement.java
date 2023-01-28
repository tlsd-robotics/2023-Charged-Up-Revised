// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.navX;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToRelativeDisplacement extends CommandBase {
  /** Creates a new DriveToRelativeDisplacement. */
  DrivetrainSubsystem drivetrain;

  //Distance Error Tolerance (m)
  final double tolerance = 0.1;

  //Initial displacements
  double xi;
  double yi;

  //Target Change
  double xc;
  double yc;

  PIDController pidX = new PIDController(1, 0, 0);
  PIDController pidY = new PIDController(1, 0, 0);

  public DriveToRelativeDisplacement(DrivetrainSubsystem drivetrain, double xDistance, double yDistance) {
    xc = xDistance;
    yc = yDistance;
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xi = -1 * navX.navX.getDisplacementX();
    yi = -1 * navX.navX.getDisplacementY();
    pidX.setTolerance(tolerance);
    pidY.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.

  double x;
  double y;

  @Override
  public void execute() {
    //The axes may need switched.
    x = navX.navX.getDisplacementX();
    y = navX.navX.getDisplacementY();
    drivetrain.drive(new ChassisSpeeds(pidX.calculate(x - xi, xc), pidY.calculate(y - yi, yc), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    //if x and y are within tolerance, return true;
    return (((x - xi) - xc <= tolerance) && ((x - xi) - xc >= 0-tolerance)) &&
           (((y - yi) - yc <= tolerance) && ((y - yi) - yc >= 0-tolerance));
    */
    return pidX.atSetpoint() && pidY.atSetpoint();
  }
}
