package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.navX;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalancingCommand extends CommandBase {

  public DrivetrainSubsystem drivetrain;

  private final double TOLERANCE = .9;

  PIDController pidX = new PIDController(0.05, 0.00, 0.2);

  public BalancingCommand(DrivetrainSubsystem Drivetrain) {
    this.drivetrain = Drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX.setTolerance(TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("Pitch",navX.navX.getPitch());

    drivetrain.drive(new ChassisSpeeds(pidX.calculate(navX.navX.getPitch(), 0), 0, 0));
    
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

