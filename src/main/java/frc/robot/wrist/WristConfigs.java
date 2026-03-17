package frc.robot.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;


public final class WristConfigs {
    private static final TalonFXConfiguration wristMotorInitialConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration wristMotorConfig = wristMotorInitialConfigs.clone()
    .withMotorOutput(
        wristMotorInitialConfigs.MotorOutput.clone()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)
    )
    .withCurrentLimits(
        wristMotorInitialConfigs.CurrentLimits.clone()
            .withStatorCurrentLimit(Amps.of(5))//40
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(5))//30
            .withSupplyCurrentLimitEnable(true)
    )
    .withSlot0(
        wristMotorInitialConfigs.Slot0.clone()
            .withKP(WristConstants.kP)
            .withKI(WristConstants.kI)
            .withKD(WristConstants.kD)
            .withKA(WristConstants.kA)
            .withKG(WristConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine)
    )
    .withFeedback(
        wristMotorInitialConfigs.Feedback.clone()
            .withSensorToMechanismRatio(WristConstants.gearBoxRatio)
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
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(40))
    );
}