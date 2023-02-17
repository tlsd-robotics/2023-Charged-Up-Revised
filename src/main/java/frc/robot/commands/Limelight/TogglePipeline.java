// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;

public class TogglePipeline extends CommandBase {
  
  private boolean bDone;
  private Limelight limelight;

  public TogglePipeline(Limelight Limelight) {
    this.limelight = Limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.togglePipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bDone;
  }
}
