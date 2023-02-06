// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autonomous.DoNothing;

public class Robot extends TimedRobot {

  public static SendableChooser<Command> sendablechooser = new SendableChooser<Command>();
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    RobotContainer m_robotContainer = new RobotContainer();
    m_robotContainer.removeNotUsedError();

    sendablechooser.setDefaultOption("Do nothing", new DoNothing());
    sendablechooser.addOption("Path Auto", m_robotContainer.autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", 4, 3)));
    SmartDashboard.putData("Autonomous", sendablechooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("X Displacement: ", navX.navX.getDisplacementX());
    SmartDashboard.putNumber("Y Displacement: ", navX.navX.getDisplacementY());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = sendablechooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
