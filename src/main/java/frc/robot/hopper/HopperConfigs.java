package frc.robot.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public final class HopperConfigs {
    private static final TalonFXConfiguration hopperMotorInitialConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration hopperMotorConfig = hopperMotorInitialConfigs.clone()
    .withMotorOutput(
        hopperMotorInitialConfigs.MotorOutput.clone()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)
    )
    .withCurrentLimits(
        hopperMotorInitialConfigs.CurrentLimits.clone()
            .withStatorCurrentLimit(Amps.of(20))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(20))
            .withSupplyCurrentLimitEnable(true)
    )
    .withSlot0(
        hopperMotorInitialConfigs.Slot0.clone()
            .withKP(HopperConstants.kP)
            .withKI(HopperConstants.kI)
            .withKD(HopperConstants.kD)
            .withKA(HopperConstants.kA)
            .withKS(HopperConstants.kS)
            .withKV(HopperConstants.kV)
    )
    .withFeedback(
        hopperMotorInitialConfigs.Feedback.clone()
            .withSensorToMechanismRatio(2.0)
            .withRotorToSensorRatio(HopperConstants.gearBoxRatio / 2.0)
    )
    .withHardwareLimitSwitch(
        hopperMotorInitialConfigs.HardwareLimitSwitch.clone()
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
        hopperMotorInitialConfigs.MotionMagic.clone()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(40))
    );
}