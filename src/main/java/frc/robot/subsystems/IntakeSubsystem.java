// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final DutyCycleOut m_intakCycleOut = new DutyCycleOut(0.0);
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(3);
  }
  public void setMotorSpeed(double speed) {
    intakeMotor.setControl(m_intakCycleOut.withOutput(speed));
  }
  public Angle getPosition() {
        return intakeMotor.getPosition(true).getValue().times(360);
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