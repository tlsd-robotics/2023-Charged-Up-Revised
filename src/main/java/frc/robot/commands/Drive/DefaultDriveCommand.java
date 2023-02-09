package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.navX;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

//Similar to the DrivetrainSubsysem, I do not know exactly how this works so I will try my best to decipher it

  //Declares different varibles and names for objects
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private final DrivetrainSubsystem drivetrain;

  //This command takes in the parameters for a drivetrain, X Supplier, y Y Supplier, and Rotation(Z axis)Supplier. this. pretty much means it will use the parameter values
  //in the command
    public DefaultDriveCommand(DrivetrainSubsystem Drivetrain,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {

        this.drivetrain = Drivetrain;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    //Not quite sure how to setup Robot-oriented movement but this is starting to make a lot of sense now.

      /*
      This takes the drive command from drivetrainSubsystem and plugs values into it. Rather than hard coded numbers like the interuppted command below, it uses suppliers.
      Suppliers are a datatype that is made to change and has built in functions to return as a datatype. The parameters above are going to be linked to a joystick value...
      so it will take the axis value from the joystick. Returns it as a double, and feeds it into the drive command. Not sure the advantage of this over just calling the drive command...
      and directly plugging joystick values into it without the need of a command at all. Regardless, it gets the the x,y,z values along with the current hyroscope rotation, and the...
      fromFieldRelativeSpeeds should do the rest to figure out how to run the motors properly. Take everything I am saying with a grain of salt I am just trying to understand this. 
      */
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainSubsystem.METERS_PER_SECOND_SCALING_FACTOR,
                        m_translationYSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainSubsystem.METERS_PER_SECOND_SCALING_FACTOR,
                        m_rotationSupplier.getAsDouble() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * (DrivetrainSubsystem.METERS_PER_SECOND_SCALING_FACTOR * 2),
                        drivetrain.getOdometryRotation()
                )
        );
    }
//When Command ends, set motors to zero so it doesn't drive off into the sunset forever.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
