// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final DutyCycleOut m_rollerCycleOut = new DutyCycleOut(0.0);
  public RollerSubsystem() {
    rollerMotor = new TalonFX(IntakeConstants.rollerMotorID) ;
  }
  public void setMotorSpeed(double speed) {
    rollerMotor.setControl(m_rollerCycleOut.withOutput(speed));
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