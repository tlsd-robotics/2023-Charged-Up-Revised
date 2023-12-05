// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.UtilityClasses.AxisBooleanSupplier;
import frc.robot.UtilityClasses.AxisSupplier;
import frc.robot.UtilityClasses.ThrottledAxisSupplier;
import frc.robot.commands.Arm.ArmMoveAtRate;
import frc.robot.commands.Arm.ArmToLength;
import frc.robot.commands.Arm.ArmToSetpoint;
import frc.robot.commands.Arm.EffectorToggle;
import frc.robot.commands.Arm.ManualArmLengthControlExtend;
import frc.robot.commands.Arm.ManualArmLengthControlRetract;
import frc.robot.commands.Drive.BalancingCommand;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.DriveToRelativeDisplacement;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmLength;
import frc.robot.subsystems.ArmSubsystem.ArmSetpoint;
import frc.robot.commands.Arm.tempArmToSetpoint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class RobotContainer {

//====================== CONTROLLER CONSTANTS ===============================
  //Joystick - ID Values - Can be found in DriverStation under the USB tab
  public static final int rightJoystickID = 1;
  // public static final int leftJoystickID = 1;
  //Joystick Buttons
  public static final int JoystickTriggerR = 1;
  // public static final int JoystickTriggerL = 1;
  // public static final int JoystickLeftInside = 4;
  public static final int JoystickRightInside = 3;
  public static final int JoystickRightOutside = 4;
  // public static final int JoystickLeftOutside = 3;
  public static final int JoystickRightBottom = 2;
  // public static final int JoystickLeftBottom = 2;
  //Gamepad
  public static final int gamepadID = 0;
  //Gamepad Buttons
  public static final int GamepadA = 2;
  public static final int GamepadB = 3;
  public static final int GamepadX = 1;
  public static final int GamepadY = 4;
  public static final int GamepadL1 = 5;
  public static final int GamepadR1 = 6;          
  public static final int GamepadR3 = 9;
  public static final int GamepadL3 = 10;
  public static final int GamepadSelect = 8;
  public static final int GamepadLeftTrigger = 7;
  public static final int GamepadRightTrigger = 8;






  //Button Board
  public static final int driverStationID = 2;
  public static final int dsLeftOneID = 9;
  public static final int dsLeftTwoID = 10;
  public static final int dsLeftThreeID = 11;
  public static final int dsLeftFourID = 12;
  public static final int dsRightOneID = 5;
  public static final int dsRightTwoID = 6;
  public static final int dsRightThreeID = 7;
  public static final int dsRightFourID = 8;
  public static final int dsSmallTopID = 3;
  public static final int dsSmallBottomID = 4;
  
//=============================================================================

  //Joystick - Creates joystick objects using the ID number from constants and the Joystick class 
  // public final Joystick leftJoy = new Joystick(leftJoystickID);
  public final static Joystick rightJoy = new Joystick(rightJoystickID);
  public final Joystick gamepad = new Joystick(gamepadID);
  public final Joystick driverStation = new Joystick(driverStationID);

  //Joystick button - Declares the names for each of the joystick buttons 
  public JoystickButton rTrigger = new JoystickButton(rightJoy, JoystickTriggerR);
  // public JoystickButton lTrigger = new JoystickButton(leftJoy, JoystickTriggerL);
  // public JoystickButton lInside = new JoystickButton(leftJoy, JoystickLeftInside);
  public JoystickButton rInside = new JoystickButton(rightJoy, JoystickRightInside);
  // public JoystickButton lOutside = new JoystickButton(leftJoy, JoystickLeftOutside);
  public JoystickButton rOutside = new JoystickButton(rightJoy, JoystickRightOutside);
  public JoystickButton rBottom = new JoystickButton(rightJoy, JoystickRightBottom);
  // public JoystickButton lBottom = new JoystickButton(leftJoy, JoystickLeftBottom);

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
  public JoystickButton gamepadLeftTrigger = new JoystickButton(gamepad, GamepadLeftTrigger);
  public JoystickButton gamepadRightTrigger = new JoystickButton(gamepad, GamepadRightTrigger);
  

  //Driver Station Button Objects
  public JoystickButton dsLeftOne =     new JoystickButton(driverStation, dsLeftOneID);
  public JoystickButton dsLeftTwo =     new JoystickButton(driverStation, dsLeftTwoID);
  public JoystickButton dsLeftThree =   new JoystickButton(driverStation, dsLeftThreeID);
  public JoystickButton dsLeftFour =    new JoystickButton(driverStation, dsLeftFourID);
  public JoystickButton dsRightOne =    new JoystickButton(driverStation, dsRightOneID);
  public JoystickButton dsRightTwo =    new JoystickButton(driverStation, dsRightTwoID);
  public JoystickButton dsRightThree = new JoystickButton(driverStation, dsRightThreeID);
  public JoystickButton dsRightFour =   new JoystickButton(driverStation, dsRightFourID);
  public JoystickButton dsSmallTop =    new JoystickButton(driverStation, dsSmallTopID);
  public JoystickButton dsSmallBottom = new JoystickButton(driverStation, dsSmallBottomID);
  public Trigger dsArmAngleControlPositive = new Trigger(new AxisBooleanSupplier(-.9, false, driverStation, AxisType.kY.value));
  public Trigger dsArmAngleControlNegative = new Trigger(new AxisBooleanSupplier(.9, true, driverStation, AxisType.kY.value));
  public Trigger dsArmControlExtend = new Trigger(new AxisBooleanSupplier(.9, true, driverStation, AxisType.kX.value));
  public Trigger dsArmControlRetract = new Trigger(new AxisBooleanSupplier(-.9, false, driverStation, AxisType.kX.value));
  
  //Event map for path following
  HashMap <String, Command> eventMap = new HashMap<String, Command>();
// change
  //create axes
  ThrottledAxisSupplier rightYAxis = new ThrottledAxisSupplier(rightJoy, AxisType.kX.value, rightJoy, 3, 1.0, -1.0, true, 0.05,  true);
  ThrottledAxisSupplier rightXAxis = new ThrottledAxisSupplier(rightJoy, AxisType.kY.value, rightJoy, 3, 1.0, -1.0, true, 0.05,  true);
  ThrottledAxisSupplier rightZAxis = new ThrottledAxisSupplier(rightJoy, AxisType.kZ.value, rightJoy, 3, 1.0, -1.0, true, 0.05,  true);

  //Instantiate Subsystems
  DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  EndEffectorSubsystem effector = new EndEffectorSubsystem();

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
    rTrigger.whileTrue(new EffectorToggle(effector));
    rInside.onTrue(Commands.runOnce(drivetrain::resetEncoders, drivetrain));
  // Gamepad
    //gamepadL1.whileTrue(new AllignToTarget(drivetrain, Limelight.limelight1));
    //gamepadR1.onTrue(new TogglePipeline(Limelight.limelight1));
  //Driver Station
    dsArmControlExtend.onTrue(new ManualArmLengthControlExtend(arm));
    dsArmControlRetract.onTrue(new ManualArmLengthControlRetract(arm));

    dsArmAngleControlPositive.whileTrue(new ArmMoveAtRate(50.0, arm));
    dsArmAngleControlNegative.whileTrue(new ArmMoveAtRate(-50.0, arm));

    /* dsLeftOne.onTrue(new ArmToSetpoint(ArmSetpoint.UPPER_CONE, arm));
    dsLeftTwo.onTrue(new ArmToSetpoint(ArmSetpoint.UPPER_CUBE, arm));
    dsLeftThree.onTrue(new ArmToSetpoint(ArmSetpoint.MID_CONE, arm));
    dsLeftFour.onTrue(new ArmToSetpoint(ArmSetpoint.MID_CUBE, arm));
    dsRightOne.onTrue(new ArmToSetpoint(ArmSetpoint.SUBSTATION, arm));
    dsRightTwo.onTrue(new ArmToSetpoint(ArmSetpoint.FRONT_FLOOR, arm));
    dsRightThree.onTrue(new ArmToSetpoint(ArmSetpoint.REAR_FLOOR, arm));
    dsRightFour.onTrue(new ArmToSetpoint(ArmSetpoint.RETRACTED, arm)); */

    dsLeftOne.onTrue(new tempArmToSetpoint(ArmSetpoint.UPPER_CONE.angleDegrees, ArmSetpoint.UPPER_CONE.length, arm));
    dsLeftTwo.onTrue(new tempArmToSetpoint(ArmSetpoint.UPPER_CUBE.angleDegrees, ArmSetpoint.UPPER_CUBE.length, arm));
    dsLeftThree.onTrue(new tempArmToSetpoint(ArmSetpoint.MID_CONE.angleDegrees, ArmSetpoint.MID_CONE.length, arm));
    dsLeftFour.onTrue(new tempArmToSetpoint(ArmSetpoint.MID_CUBE.angleDegrees, ArmSetpoint.MID_CUBE.length, arm));
    dsRightOne.onTrue(new tempArmToSetpoint(ArmSetpoint.SUBSTATION.angleDegrees, ArmSetpoint.SUBSTATION.length, arm));
    dsRightTwo.onTrue(new tempArmToSetpoint(ArmSetpoint.FRONT_FLOOR.angleDegrees, ArmSetpoint.FRONT_FLOOR.length, arm));
    //dsRightThree.onTrue(new tempArmToSetpoint(ArmSetpoint.REAR_FLOOR.angleDegrees, ArmSetpoint.REAR_FLOOR.length, arm));
    dsRightFour.onTrue(new tempArmToSetpoint(ArmSetpoint.RETRACTED.angleDegrees, ArmSetpoint.RETRACTED.length, arm));
    dsSmallTop.onTrue(new EffectorToggle(effector));

    gamepadL1.onTrue(new tempArmToSetpoint(ArmSetpoint.UPPER_CONE.angleDegrees, ArmSetpoint.UPPER_CONE.length, arm));
    gamepadR1.onTrue(new tempArmToSetpoint(ArmSetpoint.UPPER_CUBE.angleDegrees, ArmSetpoint.UPPER_CUBE.length, arm));
    gamepadX.onTrue(new tempArmToSetpoint(ArmSetpoint.SUBSTATION.angleDegrees, ArmSetpoint.SUBSTATION.length, arm));
    gamepadLeftTrigger.onTrue(new tempArmToSetpoint(ArmSetpoint.MID_CONE.angleDegrees, ArmSetpoint.MID_CONE.length, arm));
    gamepadRightTrigger.onTrue(new tempArmToSetpoint(ArmSetpoint.MID_CUBE.angleDegrees, ArmSetpoint.MID_CUBE.length, arm));
   gamepadY .onTrue(new tempArmToSetpoint(ArmSetpoint.FRONT_FLOOR.angleDegrees, ArmSetpoint.FRONT_FLOOR.length, arm));
    //dsRightThree.onTrue(new tempArmToSetpoint(ArmSetpoint.REAR_FLOOR.angleDegrees, ArmSetpoint.REAR_FLOOR.length, arm));
    gamepadB.onTrue(new tempArmToSetpoint(ArmSetpoint.RETRACTED.angleDegrees, ArmSetpoint.RETRACTED.length, arm));
    gamepadA.onTrue(new EffectorToggle(effector));






   
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

  //Factory for a command that follows a trajectory.
  public Command followTrajectory(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               drivetrain.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
          traj,
          drivetrain::getPose,
          new PIDController(1.5, 0.0, 0.0),
          new PIDController(1.5, 0.0, 0.0),
          new PIDController(0.5, 0.0, 0.0),
          drivetrain::driveFieldRelative,
          drivetrain
         )
     );
 }

  public void removeNotUsedError() {
  }
}
