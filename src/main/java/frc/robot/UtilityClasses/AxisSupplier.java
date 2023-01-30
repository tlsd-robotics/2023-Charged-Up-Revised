// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilityClasses;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;

/** Add your docs here. */
public class AxisSupplier implements DoubleSupplier {
    GenericHID hidController;
    int axisId;
    boolean squared;
    double deadzone;
    boolean inverted;

    //Creates class for an axis that can be passed to commands. Note: Deadzones are applied before squaring.
    public AxisSupplier(GenericHID HIDController, int AxisID, boolean Squared, double Deadzone, boolean Inverted) {
        hidController = HIDController;
        axisId = AxisID;
        squared = Squared;
        deadzone = Deadzone;
        inverted = Inverted;
    }
    
    public double getAsDouble () {
        if (squared) {
            double value = ApplyDeadzone(hidController.getRawAxis(axisId), deadzone);
            return (inverted ? -1 : 1) * Math.copySign(value * value, value);
        }
        else {
            return (inverted ? -1 : 1) * ApplyDeadzone(hidController.getRawAxis(axisId), deadzone);
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
