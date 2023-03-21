// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EffectorToState extends CommandBase {
  /** Creates a new EffectorToggle. */

  EndEffectorSubsystem effector;
  Boolean isOpen;

  private final double TRANSITION_TIME_SECONDS = 1;

  Timer timer = new Timer();

  public EffectorToState(boolean isOpen, EndEffectorSubsystem effector) {
    this.isOpen = isOpen;
    this.effector = effector;
    addRequirements(effector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    effector.setOpen(isOpen);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= TRANSITION_TIME_SECONDS;
  }
}
