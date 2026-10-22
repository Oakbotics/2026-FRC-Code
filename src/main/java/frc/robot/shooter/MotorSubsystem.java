// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {

  private final TalonFX tempMotor;
  private final VoltageOut voltageOut = new VoltageOut(0); 
  
  public MotorSubsystem() {
    tempMotor = new TalonFX(4);
    //configureMotors();
  }

  public void intakeFuel(double speed) {
    tempMotor.setControl(voltageOut.withOutput(speed));
  }

  public void outakeFuel(double speed) {
    tempMotor.setControl(voltageOut.withOutput(-speed));
  }

  // public void configureMotors() {
  //   tempMotor.getConfigurator().apply(configs.intakeMotorConfigs());
  // }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}