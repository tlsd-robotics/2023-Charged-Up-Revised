package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/*
 * It seems to me that the vast majority of the items in here could be placed
 * in the subystems they are relevant to, therefore decrease the complexity of this
 * file.
 */

public final class Constants {
//Constants are numbers that will not change, or if you do change them, it will make it easy to find since everything is here.
  // If you use a number more than once for something, throw it in here to make your life a lot easier.

  public class controllerConstants {
    //Joystick - ID Values - Can be found in DriverStation under the USB tab
    public static final int rightJoystick = 0;
  	public static final int leftJoystick = 1;
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
    public static final int gamepad = 3;
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
  }
    

    public class limelightConstants {
      public static final int LED_ON = 3;
      public static final int LED_OFF = 1;
      public static final int TARGET_PIPELINE = 0;
      public static final int DEFAULT_PIPELINE = 0;
      public static final int DRIVE_PIPELINE = 2;
    }
    
    //compressor
    public static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    
  // Value of motor divided by speed.
  // Eg.  / 2
  public static int METERS_PER_SECOND_DIVIDED = 1;


  /* 
  ==== Don't mind me, just storing some useful links here =====
  https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/117
  https://www.chiefdelphi.com/t/generic-swerve-library/424853/7
  https://www.chiefdelphi.com/t/some-questions-about-the-wpis-official-swerve-code/395386

  */ 

}