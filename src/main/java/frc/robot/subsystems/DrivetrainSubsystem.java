// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.navX;

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

  public static float METERS_PER_SECOND_SCALING_FACTOR = 1;

  //Calcuates Max Velocity using Motor RPM / 60. Drive Reduction is found from the Gear Ratios from the Swerve Modules. Then gets circumfrence of wheels using wheel diameter and pi
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

  //Declare Drive Modules and Odometry
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  private final SwerveDriveOdometry odometry;

  //Tab for shuffleboard data output
  final ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  public DrivetrainSubsystem() {

    // Define Drivetrain Modules
    frontLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
    .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
    .build();

    frontRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
    .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
    .build();

    backLeftModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
    .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
    .build();
  
    backRightModule = new MkSwerveModuleBuilder()
    .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0))
    .withGearRatio(SdsModuleConfigurations.MK4_L2)
    .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
    .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
    .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
    .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
    .build();

    odometry = new SwerveDriveOdometry(
      swerveKinemactics, 
      Rotation2d.fromDegrees(navX.navX.getFusedHeading()), 
      new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
    );

    tab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
    tab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
    tab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

  }

  public void zeroGyroscope() {
    odometry.resetPosition(
            Rotation2d.fromDegrees(navX.navX.getFusedHeading()),
            new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
    );
}

  //Takes chassis speeds to drive at (m/s?). Does not apply field oriented transofrms (Do it yourself)
  public void drive(final ChassisSpeeds chassisSpeeds) {
    currentSpeed = chassisSpeeds;
  }

  @Override
  public void periodic() {
    //TODO: Consider Swerve Optimization - 
    
    odometry.update(
                Rotation2d.fromDegrees(navX.navX.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

    final SwerveModuleState[] states = swerveKinemactics.toSwerveModuleStates(currentSpeed);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }

  // =============== Getter Methods ======================

public Rotation2d getRotation() {
    return odometry.getPoseMeters().getRotation();
}

public SwerveDriveOdometry getOdometry() {
    return odometry;
}

public Pose2d getPose2d() {
    return odometry.getPoseMeters();
}

//PATH FOLLOWING -------------------------------------------------------------------------

//Creates a path following command given a path. This could be done outside of the subsystem every time a path command is desired.
//The docs demonstrate a version that outputs module states via swerve kinematics requiring direct access to private drivetrain objects,
//and therefore must be in the drivetrain subsystem. That is not the case here.
Command getPathFollowingCommand(PathPlannerTrajectory Path) {
        return new PPSwerveControllerCommand(
                Path, //Takes a path, created with path planner
                this::getPose2d, //Takes the current pose of the robot
                new PIDController(0, 0, 0), //X          PID controllers for some part of the path following.
                new PIDController(0, 0, 0), //Y          these will need to be tuned. Further research required.
                new PIDController(0, 0, 0), //Rotation
                this::drive, //Method that takes the generated chassis speeds and makes the robot do robot things. (Consumer<ChassisSpeeds>)
                false, //Can automatically perform transformations on paths based on alliance color. Might be beneficial to enable later.
                this //Command requires the drivetrain subsystem. This could cause confict later if we want to have events that require the drivetrain -
        );           //such as loading a game piece - mid path.
}

//Pathplannerlib docs: https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage 

}
