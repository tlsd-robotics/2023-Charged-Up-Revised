// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.UtilityClasses.AxisSupplier;
import frc.robot.UtilityClasses.ThrottledAxisSupplier;
import frc.robot.commands.Drive.BalancingCommand;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.DriveRelativeDistance;
import frc.robot.commands.Drive.DriveToRelativeDisplacement;
import frc.robot.subsystems.DrivetrainSubsystem;
public class RobotContainer {

//====================== CONTROLLER CONSTANTS ===============================
  //Joystick - ID Values - Can be found in DriverStation under the USB tab
  public static final int rightJoystickID = 0;
  public static final int leftJoystickID = 1;
  //Joystick Buttons
  public static final int JoystickTriggerR = 1;
  public static final int JoystickTriggerL = 1;
  public static final int JoystickLeftInside = 4;
  public static final int JoystickRightInside = 3;
  public static final int JoystickRightOutside = 4;
  public static final int JoystickLeftOutside = 3;
  public static final int JoystickRightBottom = 2;
  public static final int JoystickLeftBottom = 2;
  //Gamepad
  public static final int gamepadID = 3;
  //Gamepad Buttons
  public static final int GamepadA = 1;
  public static final int GamepadB = 2;
  public static final int GamepadX = 3;
  public static final int GamepadY = 4;
  public static final int GamepadL1 = 5;
  public static final int GamepadR1 = 6;          
  public static final int GamepadR3 = 9;
  public static final int GamepadL3 = 10;
  public static final int GamepadSelect = 8;
//=============================================================================

  //Joystick - Creates joystick objects using the ID number from constants and the Joystick class 
  public final Joystick leftJoy = new Joystick(leftJoystickID);
  public final static Joystick rightJoy = new Joystick(rightJoystickID);
  public final Joystick gamepad = new Joystick(gamepadID);

  //Joystick button - Declares the names for each of the joystick buttons 
  public JoystickButton rTrigger = new JoystickButton(rightJoy, JoystickTriggerR);
  public JoystickButton lTrigger = new JoystickButton(leftJoy, JoystickTriggerL);
  public JoystickButton lInside = new JoystickButton(leftJoy, JoystickLeftInside);
  public JoystickButton rInside = new JoystickButton(rightJoy, JoystickRightInside);
  public JoystickButton lOutside = new JoystickButton(leftJoy, JoystickLeftOutside);
  public JoystickButton rOutside = new JoystickButton(rightJoy, JoystickRightOutside);
  public JoystickButton rBottom = new JoystickButton(rightJoy, JoystickRightBottom);
  public JoystickButton lBottom = new JoystickButton(leftJoy, JoystickLeftBottom);

  //GamePad - Declares the names for each of the gamepad buttons
  public JoystickButton gamepadX = new JoystickButton(gamepad, GamepadX);
  public JoystickButton gamepadA = new JoystickButton(gamepad, GamepadA);
  public JoystickButton gamepadY = new JoystickButton(gamepad, GamepadY);
  public JoystickButton gamepadB = new JoystickButton(gamepad, GamepadB);
  //public JoystickButton gamepadStart = new JoystickButton(gamepad, GamepadStart); Not in controllerConstants
  public JoystickButton gamepadSelect  = new JoystickButton(gamepad, GamepadSelect);
  public JoystickButton gamepadL1 = new JoystickButton(gamepad, GamepadL1);
  public JoystickButton gamepadR1 = new JoystickButton(gamepad, GamepadR1);
  public JoystickButton gamepadR3 = new JoystickButton(gamepad, GamepadR3);
  public JoystickButton gamepadL3 = new JoystickButton(gamepad, GamepadL3);

  //Event map for path following
  HashMap <String, Command> eventMap = new HashMap<String, Command>();

  //create axes
  AxisSupplier rightYAxis = new AxisSupplier(rightJoy, AxisType.kX.value, true, 0.05, true);
  AxisSupplier rightXAxis = new AxisSupplier(rightJoy, AxisType.kY.value, true, 0.05, true);
  AxisSupplier rightZAxis = new AxisSupplier(rightJoy, AxisType.kZ.value, true, 0.05, true);

  //Instantiate Subsystems
  DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  public RobotContainer() {
    //NOTE: Mapping is now done in DefualtDriveCommand
    drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, rightXAxis, rightYAxis, rightZAxis));

    configureBindings(); 
    configureEventMap();
  }

  private void configureBindings() {

  // ======================= Button Commands ============================
  // Joysticks
    rBottom.onTrue(Commands.runOnce(drivetrain::resetOdometry, drivetrain));
    rOutside.onTrue(new BalancingCommand(drivetrain));
    rTrigger.whileTrue(new DriveRelativeDistance(drivetrain, 1.0, 0, 0));
  // Gamepad
    //gamepadL1.whileTrue(new AllignToTarget(drivetrain, Limelight.limelight1));
    //gamepadR1.onTrue(new TogglePipeline(Limelight.limelight1));
    
  }

  //==================== Path Following Event Map ========================
  private void configureEventMap() {
    eventMap.put("BalancingCommand", new BalancingCommand(drivetrain));
  }

  //Factory for a command that executes a path group with events
  //Path groups break up paths into sections between which events
  //can be executed without conflict. This builds a command handling
  //this logic.
  public SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose,
    drivetrain::resetOdometry,
    new PIDConstants(1.5, 0.0, 0.0),
    new PIDConstants(0.5, 0.0, 0.0),
    drivetrain::driveFieldRelative,
    eventMap,
    drivetrain
  );

  public void removeNotUsedError() {
  }
}
