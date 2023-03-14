// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Limelight;
import frc.robot.LimelightPipeline;
import frc.robot.commands.Arm.ArmToSetpoint;
import frc.robot.commands.Arm.EffectorToState;
import frc.robot.commands.Limelight.AlignTranslateY;
import frc.robot.commands.Limelight.DriveToDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndDropAtSetpoint extends SequentialCommandGroup {
  /** Creates a new DropAtSetpoint. */
  public AlignAndDropAtSetpoint(ArmSetpoint setpoint, double metersFromTarget, double targetHeightMeters, ArmSubsystem arm, DrivetrainSubsystem drivetrain, EndEffectorSubsystem effector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> {Limelight.limelightFront.setPipeline(Limelight.reflective);}), 
      new AlignTranslateY(drivetrain, Limelight.limelightFront, true),
      new DriveToDistance(drivetrain, Limelight.limelightFront, metersFromTarget, 0.5, targetHeightMeters, false),
      new ArmToSetpoint(setpoint, arm),
      new EffectorToState(true, effector)
    );
  }
}
