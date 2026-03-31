// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  private final TalonFX kickerMotor;
  private final VoltageOut speedRequest = new VoltageOut(0);
  public KickerSubsystem() {
    kickerMotor = new TalonFX(ShooterConstants.kickerMotorID);

    kickerMotor.getVelocity().setUpdateFrequency(100);
    kickerMotor.getStatorCurrent().setUpdateFrequency(100);
  }

  public void setKickerSpeed(double speed){
    kickerMotor.setControl(speedRequest.withOutput(speed));
  }

  public double getVelovcityRps(){
    return kickerMotor.getVelocity().getValueAsDouble();
  }

  public double getStatorCurrentAmps(){
    return kickerMotor.getStatorCurrent().getValueAsDouble();
  }

  public void stop() {
    setKickerSpeed(0.0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}