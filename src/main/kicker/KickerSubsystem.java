// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kicker;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {

  private final TalonFX kickerMotor;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
  private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
  
  /** Creates a new ExampleSubsystem. */
  public KickerSubsystem() {
    kickerMotor = new TalonFX(KickerConstants.kickerMotorID);
  }

  public void kickFuel(double speed){
    kickerMotor.setControl(dutyCycle.withOutput(speed));
  }

  public void reverseKicker(double speed){
    kickerMotor.setControl(dutyCycle.withOutput(-speed));
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