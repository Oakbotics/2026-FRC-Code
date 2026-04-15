// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0).withMaxAbsDutyCycle(0.9);
  private final IntakeConfigs configs;
  
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
    configs = new IntakeConfigs();
    configureMotors();
  }

  public void intakeFuel(double amps) {
    intakeMotor.setControl(torqueCurrentFOC.withOutput(amps));
  }

  public void outakeFuel(double amps) {
    intakeMotor.setControl(torqueCurrentFOC.withOutput(-amps));
  }

  public void configureMotors() {
    intakeMotor.getConfigurator().apply(configs.intakeMotorConfigs());
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}