// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.DriveToRelativeDisplacement;
import frc.robot.commands.Drive.driveToDistance;
import frc.robot.commands.navX.zeroNavXYaw;
import frc.robot.commands.navX.zeroNavxDisplacement;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

  //Joystick - Creates joystick objects using the ID number from constants and the Joystick class 
  public final Joystick leftJoy = new Joystick(Constants.leftJoystick);
  public final static Joystick rightJoy = new Joystick(Constants.rightJoystick);
  public final Joystick gamepad = new Joystick(Constants.gamepad);

  //Joystick button - Declares the names for each of the joystick buttons 
  public JoystickButton rTrigger = new JoystickButton(rightJoy, Constants.JoystickTriggerR);
  public JoystickButton lTrigger = new JoystickButton(leftJoy, Constants.JoystickTriggerL);
  public JoystickButton lInside = new JoystickButton(leftJoy, Constants.JoystickLeftInside);
  public JoystickButton rInside = new JoystickButton(rightJoy, Constants.JoystickRightInside);
  public JoystickButton lOutside = new JoystickButton(leftJoy, Constants.JoystickLeftOutside);
  public JoystickButton rOutside = new JoystickButton(rightJoy, Constants.JoystickRightOutside);
  public JoystickButton rBottom = new JoystickButton(rightJoy, Constants.JoystickRightBottom);
  public JoystickButton lBottom = new JoystickButton(leftJoy, Constants.JoystickLeftBottom);

  //GamePad - Declares the names for each of the gamepad buttons
  public JoystickButton gamepadX = new JoystickButton(gamepad, Constants.GamepadX);
  public JoystickButton gamepadA = new JoystickButton(gamepad, Constants.GamepadA);
  public JoystickButton gamepadY = new JoystickButton(gamepad, Constants.GamepadY);
  public JoystickButton gamepadB = new JoystickButton(gamepad, Constants.GamepadB);
  //public JoystickButton gamepadStart = new JoystickButton(gamepad, Constants.GamepadStart); Not in Constants
  public JoystickButton gamepadSelect  = new JoystickButton(gamepad, Constants.GamepadSelect);
  public JoystickButton gamepadL1 = new JoystickButton(gamepad, Constants.GamepadL1);
  public JoystickButton gamepadR1 = new JoystickButton(gamepad, Constants.GamepadR1);
  public JoystickButton gamepadR3 = new JoystickButton(gamepad, Constants.GamepadR3);
  public JoystickButton gamepadL3 = new JoystickButton(gamepad, Constants.GamepadL3);

  //create axes
  AxisSupplier rightYAxis = new AxisSupplier(rightJoy, AxisType.kX.value, true, 0.05, false);
  AxisSupplier rightXAxis = new AxisSupplier(rightJoy, AxisType.kY.value, true, 0.05, false);
  AxisSupplier rightZAxis = new AxisSupplier(rightJoy, AxisType.kZ.value, true, 0.05, true);

  //Instantiate Subsystems
  DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  public RobotContainer() {
    configureBindings();
    //NOTE: Mapping is now done in DefualtDriveCommand
    drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, rightXAxis, rightYAxis, rightZAxis));
  }

  private void configureBindings() {
    rBottom.onTrue(new zeroNavXYaw());
    rOutside.onTrue(new zeroNavxDisplacement());
    rTrigger.whileTrue(new DriveToRelativeDisplacement(drivetrain, 1, 0));
    rInside.whileTrue(new driveToDistance(drivetrain, 1, 1));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
