// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX kickerMotor;
  private final DutyCycleOut speedRequest = new DutyCycleOut(0);
  /** Creates a new ExampleSubsystem. */
  public KickerSubsystem() {
    kickerMotor = new TalonFX(ShooterConstants.kickerMotorID);

  }
  public void setKickerSpeed(double speed){
    kickerMotor.setControl(speedRequest.withOutput(speed));
  }


  public void stop() {
    setKickerSpeed(0.0);
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