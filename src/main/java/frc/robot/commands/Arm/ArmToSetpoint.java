// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmLength;
import frc.robot.subsystems.ArmSubsystem.ArmSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToSetpoint extends SequentialCommandGroup {
  /** Creates a new ArmToSetpoint. */
  //Moves arm to a given angle, then to a given length.
  public ArmToSetpoint(double angle, ArmLength length, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (arm.getCurrentArmLength().ordinal() < length.ordinal()) {
      addCommands(new ArmToAngle(angle, arm), new ArmToLength(length, arm));
    }
    else if (arm.getCurrentArmLength().ordinal() > length.ordinal()) {
      addCommands(new ArmToLength(length, arm), new ArmToAngle(angle, arm));
    }
    else {
      addCommands(new ArmToAngle(angle, arm));
    }
  }

  public ArmToSetpoint(ArmSetpoint setpoint, ArmSubsystem arm) {
    if (arm.getCurrentArmLength().ordinal() < setpoint.length.ordinal()) {
      addCommands(new ArmToAngle(setpoint.angleDegrees, arm), new ArmToLength(setpoint.length, arm));
    }
    else if (arm.getCurrentArmLength().ordinal() > setpoint.length.ordinal()) {
      addCommands(new ArmToLength(setpoint.length, arm), new ArmToAngle(setpoint.angleDegrees, arm));
    }
    else {
      addCommands(new ArmToAngle(setpoint.angleDegrees, arm));
    }
  }
}
