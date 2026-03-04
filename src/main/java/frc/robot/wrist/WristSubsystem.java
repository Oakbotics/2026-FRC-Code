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
  
    public final int wristMotorId = 3;
    public final int wristEncoderId = 4;
    private final TalonFX wristMotor;
    public int kNumConfigAttempts = 5;
    final CANcoder encoder;
    private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0);
    public WristSubsystem() {
  
      final double magnetOffset = 0.0;
      final SensorDirectionValue SensorDirectionValue = null;
          
      encoder = new CANcoder(wristEncoderId);
      CANcoderConfiguration config = new CANcoderConfiguration();
          
      //config.MagnetSensor.SensorDirection = SENSOR_INVERTED;
      config.MagnetSensor.MagnetOffset = magnetOffset;
      config.MagnetSensor.SensorDirection = SensorDirectionValue;
    encoder.getConfigurator().apply(config);


    //define wrist motor and apply configs
    wristMotor = new TalonFX(wristMotorId);
    for (int i = 0; i < 5; ++i) {
            var status = wristMotor.getConfigurator().apply(wristMotorConfig);
            if (status.isOK()) break;
    }
    wristMotor.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
  }
  /** Configs for wristMotor. */
  private static final TalonFXConfiguration wristMotorInitialConfigs = new TalonFXConfiguration();
  private final TalonFXConfiguration wristMotorConfig = wristMotorInitialConfigs.clone()
      .withMotorOutput(
          wristMotorInitialConfigs.MotorOutput.clone()
              .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(
          wristMotorInitialConfigs.CurrentLimits.clone()
              .withStatorCurrentLimit(Amps.of(120))
              .withStatorCurrentLimitEnable(true)
      )
      .withSlot0(
          wristMotorInitialConfigs.Slot0.clone()
              .withKP(500)
              .withKI(0)
              .withKD(1)
              .withKS(0)
              .withKV(2)
              .withKA(0)
              .withKG(0)
              .withGravityType(GravityTypeValue.Arm_Cosine)
      )
      .withFeedback(
          wristMotorInitialConfigs.Feedback.clone()
              .withSensorToMechanismRatio(2)
      )
      .withHardwareLimitSwitch(
          wristMotorInitialConfigs.HardwareLimitSwitch.clone()
              .withForwardLimitEnable(true)
        
              .withForwardLimitAutosetPositionEnable(false)
              .withForwardLimitRemoteSensorID(0)
              .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
              .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
              .withReverseLimitAutosetPositionEnable(false)
              .withReverseLimitEnable(true)
              .withReverseLimitRemoteSensorID(0)
              .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
              .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
      )
      .withMotionMagic(
          wristMotorInitialConfigs.MotionMagic.clone()
              .withMotionMagicCruiseVelocity(RotationsPerSecond.of(256))
              .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1000))
      );
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
      
          //SmartDashboard.putNumber("target", setpoint.get().target.magnitude());
          setpointRequest.withPosition(newPos);
          wristMotor.setControl(setpointRequest);
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