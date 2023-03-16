// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilityClasses;

import java.util.function.DoubleSupplier;

/** Add your docs here. */

import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class ThrottledAxisSupplier implements DoubleSupplier {
    GenericHID hidController;
    int axisId;
    GenericHID throttleHidController;
    int throttleAxisId;
    double throttleMin;
    double throttleMax;
    boolean squared;
    double deadzone;
    boolean inverted;


    //Creates class for an axis that can be passed to commands. Note: Deadzones are applied before squaring, throttling is applied after both.
    public ThrottledAxisSupplier(GenericHID HIDController, int AxisID, GenericHID ThrottleHIDController, int ThrottleAxisID, 
                          double ThrottleMin, double ThrottleMax, boolean Squared, double Deadzone, boolean inverted) {
        hidController = HIDController;
        axisId = AxisID;
        throttleHidController = ThrottleHIDController;
        throttleAxisId = ThrottleAxisID;
        throttleMin = ThrottleMin;
        throttleMax = ThrottleMax;
        squared = Squared;
        deadzone = Deadzone;
        this.inverted = inverted;
    }
    
    public double getAsDouble () {
        if (squared) {
            double value = ApplyDeadzone(hidController.getRawAxis(axisId), deadzone);
            return (inverted ? -1 : 1) * Util.map(throttleHidController.getRawAxis(throttleAxisId), throttleMin, throttleMax, 0, 1) * 
                   Math.copySign(value * value, value);
        }
        else {
            return (inverted ? -1 : 1) * Util.map(throttleHidController.getRawAxis(throttleAxisId), throttleMin, throttleMax, 0, 1) * 
                   ApplyDeadzone(hidController.getRawAxis(axisId), deadzone);
        }
    }

    private double ApplyDeadzone(double Value, double Deadzone) {
        if (Math.abs(Value) <= Deadzone) {
            return 0;
        }
        else {
            return Value;
        }
    }
}

