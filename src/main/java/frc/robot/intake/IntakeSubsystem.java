// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
  private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
  
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
  }

  public void intakeFuel(double speed){
    intakeMotor.setControl(dutyCycle.withOutput(speed));
  }

  public void outakeFuel(double speed){
    intakeMotor.setControl(dutyCycle.withOutput(-speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}