// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

  private DoubleSolenoid effectorSolenoid = new DoubleSolenoid(3, PneumaticsModuleType.REVPH, 0, 16);

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    //effectorSolenoid.set(Value.kForward); // Start in open configuration
  }

  

  public void open() {
    effectorSolenoid.set(Value.kForward);
  }

  public void close() {
    effectorSolenoid.set(Value.kReverse);
  }

  public boolean isOpen() {
    return effectorSolenoid.get() == Value.kForward;
  }

  public void setOpen(boolean isOpen) {
    effectorSolenoid.set(isOpen ? Value.kForward : Value.kReverse);
  }

  @Override
  public void periodic() {}
}
