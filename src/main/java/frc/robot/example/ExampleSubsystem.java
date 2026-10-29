// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(0);
  private final VoltageOut voltageOut = new VoltageOut(0); 
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public ExampleSubsystem() {

  }

  public void spinMotor(double motorSpeed) {
    intakeMotor.setControl(voltageOut.withOutput(motorSpeed));
    //intakeMotor.setControl(dutyCycleOut.withOutput(motorSpeed));
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}