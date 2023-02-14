package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class navX {
    //keep the navx here to access it from anywhere.
    //Methods for interacting with the navx can be added to 
    //this class
    
    //Creates Navx object using AHRS class and the MXP port on the RoboRio
    public static AHRS navX = new AHRS(Port.kMXP, (byte) 200);
}
