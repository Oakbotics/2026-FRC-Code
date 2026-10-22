// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  private final TalonFX exampleMotor;
  private final VoltageOut voltageOut; 
  
  public ExampleSubsystem() {
    exampleMotor = new TalonFX(1);
    voltageOut = new VoltageOut(0); 
  }

  public void motorForward(double speed) {
    exampleMotor.setControl(voltageOut.withOutput(speed));
  }

  public void motorBackward(double speed) {
    exampleMotor.setControl(voltageOut.withOutput(-speed));
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}