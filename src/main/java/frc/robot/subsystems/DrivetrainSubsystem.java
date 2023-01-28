// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  //DRIVETRAIN CONSTANTS:----------------------------------------------------------------------
  //-------------------------------------------------------------------------------------------
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.5);
  public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.5);

  //CAN ID of the SpeedContoller linked to the Motor in question
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 27; 
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 28; 
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
  //The offset recorded in shuffleboard when lining up the wheels forward. 
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(271.7578454887965); 
  //Same format as above
  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 21; 
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 22; 
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(71.89453705260837);  

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 25; 
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 29; 
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(315.8789004325979 - 90);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 23; 
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 24; 
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10; 
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(6.240234995547485);   

  //This varible represents the MAX_VOLTAGE the motors will use when operating. Lower means slower, but may help with brownouts
  public static final double MAX_VOLTAGE = 12.0;

  //Calcuates Max Velocity using Motor RPM / 60. Drive Reduction is found from the Gear Ratios from the Swerve Modules. Then gets cirfumfrence of wheels using wheel diameter and pi
  public final static double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 * 
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  //Calucates the max speed of the Angle Motors
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  //-------------------------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------------------------

  ChassisSpeeds currentSpeed = new ChassisSpeeds(0,0,0); //stores current movement

  //Does swerve drive computations. Will return SwerveModuleStates for each module, in the
  //order they are entered in the constructor.
  SwerveDriveKinematics swerveKinemactics = new SwerveDriveKinematics( //this is likely why the axes are messed up. (x = +front/back, y = +left/right)  
                                                                       //TODO: Switch TRACKWIDTH and WHEELBASE
    // Front Left                                                                                  ^
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),  //  X
    // Front right                                                                             Y___|
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  //Declare Drive Modules
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  //Tab for shuffleboard data output
  final ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  public DrivetrainSubsystem() {
    // Define Drivetrain Modules
    m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(
      // This parameter is optional, but will allow you to see the current s
      tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
      Mk4SwerveModuleHelper.GearRatio.L2, // This can either be STANDARD or FAST depending on your gear configur
      FRONT_LEFT_MODULE_DRIVE_MOTOR, // This is the ID of the drive motor
      FRONT_LEFT_MODULE_STEER_MOTOR, // This is the ID of the steer motor
      FRONT_LEFT_MODULE_STEER_ENCODER, // This is the ID of the steer encoder
      FRONT_LEFT_MODULE_STEER_OFFSET // This is how much the steer encoder is offset from true zero (In our
    );
    m_frontRightModule = Mk4SwerveModuleHelper.createNeo(
      tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
      Mk4SwerveModuleHelper.GearRatio.L2,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_OFFSET
    );
    m_backLeftModule = Mk4SwerveModuleHelper.createNeo(
      tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
      Mk4SwerveModuleHelper.GearRatio.L2,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_OFFSET
    );
    m_backRightModule = Mk4SwerveModuleHelper.createNeo(
      tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
      Mk4SwerveModuleHelper.GearRatio.L2,
      BACK_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  //Takes chassis speeds to drive at (m/s?). Does not apply field oriented transofrms (Do it yourself)
  public void drive(final ChassisSpeeds chassisSpeeds) {
    currentSpeed = chassisSpeeds;
  }

  @Override
  public void periodic() {
    //TODO: Consider Swerve Optimization - 
    //https://www.chiefdelphi.com/t/some-questions-about-the-wpis-official-swerve-code/395386
    final SwerveModuleState[] states = swerveKinemactics.toSwerveModuleStates(currentSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}
