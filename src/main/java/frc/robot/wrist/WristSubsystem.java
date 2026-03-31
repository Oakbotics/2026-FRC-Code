// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

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
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.85;
    encoder.getConfigurator().apply(config);


  wristMotor = new TalonFX(m_WristConstants.wristMotorId);
  for (int i = 0; i < 5; ++i) {
    var status = wristMotor.getConfigurator().apply(m_WristConfigs.wristMotorConfig);
    if (status.isOK()) break;
  }

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
    double rotations = newPos.in(Rotations);
    
    wristMotor.setControl(setpointRequest.withPosition(rotations));
   
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position (deg)", getPosition().in(Degrees));

  }

  @Override
  public void simulationPeriodic() {}
}