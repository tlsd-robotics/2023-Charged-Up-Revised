package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.navX;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalancingCommand extends CommandBase {
  /** Creates a new BalancingCommand. */
  private double robotAngle;
  public DrivetrainSubsystem driving;
  PIDController anglePid = new PIDController(0.05, 0.01, 0);

  public BalancingCommand(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.driving = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pitch",robotAngle);
    robotAngle = navX.navX.getPitch();
    //driving.drive(new ChassisSpeeds(anglePid.calculate(robotAngle, 0), 0, 0));
    if (robotAngle < .9 && robotAngle > -.9){
    
    }
    else { 
      driving.drive(new ChassisSpeeds(anglePid.calculate(robotAngle, 0), 0, 0));
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

