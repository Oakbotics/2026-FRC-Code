package frc.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class ElevatorConfigs {

      /** Configs common across all motors. */
    private static final TalonFXConfiguration motorInitialConfigs = new TalonFXConfiguration();

    /** Configs common across just the leader motors. */
    private static final TalonFXConfiguration leaderInitialConfigs = motorInitialConfigs.clone();

    /** Configs for {@link #motor_id_1}. */
    public final TalonFXConfiguration motor_id_1Configs = leaderInitialConfigs.clone()
        .withMotorOutput(
            leaderInitialConfigs.MotorOutput.clone()
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withCurrentLimits(
            leaderInitialConfigs.CurrentLimits.clone()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        )
        .withSlot0(
            leaderInitialConfigs.Slot0.clone()
                .withKP(ElevatorConstants.kP)
                .withKI(ElevatorConstants.kI)
                .withKD(ElevatorConstants.kD)
                .withKS(ElevatorConstants.kS)
                .withKV(ElevatorConstants.kV)
                .withKA(ElevatorConstants.kA)
                .withKG(ElevatorConstants.kG)
                .withGravityType(GravityTypeValue.Elevator_Static)
        )
        .withFeedback(
            leaderInitialConfigs.Feedback.clone()
                .withSensorToMechanismRatio(ElevatorConstants.kGearRatio)
        )
        .withHardwareLimitSwitch(
            leaderInitialConfigs.HardwareLimitSwitch.clone()
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
            leaderInitialConfigs.MotionMagic.clone()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(8.88888888888889))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(44.44444444444444))
        );

    /** Configs for {@link #motor_id_2}. */
    public final TalonFXConfiguration motor_id_2Configs = motorInitialConfigs.clone()
        .withMotorOutput(
            motorInitialConfigs.MotorOutput.clone()
                .withNeutralMode(NeutralModeValue.Coast)
        )
        .withCurrentLimits(
            motorInitialConfigs.CurrentLimits.clone()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        )
        .withSlot0(
            motorInitialConfigs.Slot0.clone()
                .withKP(36)
                .withKI(0)
                .withKD(0)
                .withKS(0.2)
                .withKV(1.08)
                .withKA(0)
                .withKG(0)
                .withGravityType(GravityTypeValue.Elevator_Static)
        )
        .withFeedback(
            motorInitialConfigs.Feedback.clone()
                .withSensorToMechanismRatio(9)
        )
        .withHardwareLimitSwitch(
            motorInitialConfigs.HardwareLimitSwitch.clone()
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
            motorInitialConfigs.MotionMagic.clone()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(8.88888888888889))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(44.44444444444444))
        );
    
}
