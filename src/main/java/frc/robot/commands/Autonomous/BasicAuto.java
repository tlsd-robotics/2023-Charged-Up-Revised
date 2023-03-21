// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmToSetpoint;
import frc.robot.commands.Arm.EffectorToState;
import frc.robot.commands.Arm.tempArmToSetpoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmLength;
import frc.robot.subsystems.ArmSubsystem.ArmSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public BasicAuto(ArmSubsystem arm, EndEffectorSubsystem effector, Command driveCommand) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new EffectorToState(false, effector), 
      new tempArmToSetpoint(ArmSetpoint.FRONT_FLOOR.angleDegrees, ArmSetpoint.FRONT_FLOOR.length, arm), 
      new EffectorToState(true, effector),
      new tempArmToSetpoint(ArmSetpoint.RETRACTED.angleDegrees, ArmSetpoint.RETRACTED.length, arm),
      driveCommand
    );
  }
}
