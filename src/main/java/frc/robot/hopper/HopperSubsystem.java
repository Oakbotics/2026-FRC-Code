// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hopper;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

  private final TalonFX hopperMotor;
  private final HopperConfigs configs;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
  
  /** Creates a new ExampleSubsystem. */
  public HopperSubsystem() {
    hopperMotor = new TalonFX(HopperConstants.hopperMotorID);
    configs = new HopperConfigs();

    configureMotors();
  }

  public void hopFuel(double speed) {
    hopperMotor.setControl(dutyCycle.withOutput(-speed));
  }

  public void outHopFuel(double speed) {
    hopperMotor.setControl(dutyCycle.withOutput(speed));
  }

  public void feedTowardShooter(double percent){
    hopFuel(percent);
  }

  public void reverseFromShooter(double percent){
    outHopFuel(percent);
  }

  public void stop(){
    hopperMotor.setControl(dutyCycle.withOutput(0));
  }

  public double getVelovcityRps(){
    return hopperMotor.getVelocity().getValueAsDouble();
  }

  public double getStatorCurrentAmps(){
    return hopperMotor.getStatorCurrent().getValueAsDouble();
  }

  public void configureMotors() {
    hopperMotor.getConfigurator().apply(configs.hopperMotorConfigs());
    hopperMotor.getVelocity().setUpdateFrequency(100);
    hopperMotor.getStatorCurrent().setUpdateFrequency(100);
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