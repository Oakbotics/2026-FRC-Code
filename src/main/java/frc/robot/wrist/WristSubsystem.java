// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  
  /** Creates a new ExampleSubsystem. */
  
  
  private final TalonFX wristMotor;
  public int kNumConfigAttempts = 5;
  final CANcoder encoder;
  public final WristConfigs m_WristConfigs = new WristConfigs();
  public final WristConstants m_WristConstants = new WristConstants();
  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0);

  public WristSubsystem() {
    encoder = new CANcoder(m_WristConstants.wristEncoderId);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = m_WristConstants.magnetOffset;
    config.MagnetSensor.SensorDirection = m_WristConstants.sensorDirectionValue;
    encoder.getConfigurator().apply(config);


  //define wrist motor and apply configs
  wristMotor = new TalonFX(m_WristConstants.wristMotorId);
  for (int i = 0; i < 5; ++i) {
    var status = wristMotor.getConfigurator().apply(m_WristConfigs.wristMotorConfig);
    if (status.isOK()) break;
  }
  wristMotor.setPosition(encoder.getAbsolutePosition().getValueAsDouble() * WristConstants.gearBoxRatio);
  }
  /** Configs for wristMotor. */
  
  public double getVolts() {
    return wristMotor.getMotorVoltage(true).getValueAsDouble();
  }
  public Angle getPosition() {
    return wristMotor.getPosition(true).getValue();
  }
  public Command holdPosition() {
    return runOnce(() ->
      setpointRequest.withPosition(getPosition())
      ).andThen(run(() -> {
        wristMotor.setControl(setpointRequest);
      }));
    }

  public void goToSetpoint(Angle newPos) {
    setpointRequest.withPosition(newPos);
    wristMotor.setControl(setpointRequest);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position (deg)", getPosition().in(Degrees));
    SmartDashboard.putNumber("Wrist Voltage", getVolts());
    SmartDashboard.putNumber("Wrist Current", wristMotor.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}