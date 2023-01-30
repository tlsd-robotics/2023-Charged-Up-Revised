// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.controllerConstants;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.DriveToRelativeDisplacement;
import frc.robot.commands.navX.zeroNavxDisplacement;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

  //Joystick - Creates joystick objects using the ID number from constants and the Joystick class 
  public final Joystick leftJoy = new Joystick(controllerConstants.leftJoystick);
  public final static Joystick rightJoy = new Joystick(controllerConstants.rightJoystick);
  public final Joystick gamepad = new Joystick(controllerConstants.gamepad);

  //Joystick button - Declares the names for each of the joystick buttons 
  public JoystickButton rTrigger = new JoystickButton(rightJoy, controllerConstants.JoystickTriggerR);
  public JoystickButton lTrigger = new JoystickButton(leftJoy, controllerConstants.JoystickTriggerL);
  public JoystickButton lInside = new JoystickButton(leftJoy, controllerConstants.JoystickLeftInside);
  public JoystickButton rInside = new JoystickButton(rightJoy, controllerConstants.JoystickRightInside);
  public JoystickButton lOutside = new JoystickButton(leftJoy, controllerConstants.JoystickLeftOutside);
  public JoystickButton rOutside = new JoystickButton(rightJoy, controllerConstants.JoystickRightOutside);
  public JoystickButton rBottom = new JoystickButton(rightJoy, controllerConstants.JoystickRightBottom);
  public JoystickButton lBottom = new JoystickButton(leftJoy, controllerConstants.JoystickLeftBottom);

  //GamePad - Declares the names for each of the gamepad buttons
  public JoystickButton gamepadX = new JoystickButton(gamepad, controllerConstants.GamepadX);
  public JoystickButton gamepadA = new JoystickButton(gamepad, controllerConstants.GamepadA);
  public JoystickButton gamepadY = new JoystickButton(gamepad, controllerConstants.GamepadY);
  public JoystickButton gamepadB = new JoystickButton(gamepad, controllerConstants.GamepadB);
  //public JoystickButton gamepadStart = new JoystickButton(gamepad, controllerConstants.GamepadStart); Not in controllerConstants
  public JoystickButton gamepadSelect  = new JoystickButton(gamepad, controllerConstants.GamepadSelect);
  public JoystickButton gamepadL1 = new JoystickButton(gamepad, controllerConstants.GamepadL1);
  public JoystickButton gamepadR1 = new JoystickButton(gamepad, controllerConstants.GamepadR1);
  public JoystickButton gamepadR3 = new JoystickButton(gamepad, controllerConstants.GamepadR3);
  public JoystickButton gamepadL3 = new JoystickButton(gamepad, controllerConstants.GamepadL3);

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
    rBottom.onTrue(new RunCommand(drivetrain::zeroGyroscope));
    rOutside.onTrue(new zeroNavxDisplacement());
    rTrigger.whileTrue(new DriveToRelativeDisplacement(drivetrain, 1, 0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
