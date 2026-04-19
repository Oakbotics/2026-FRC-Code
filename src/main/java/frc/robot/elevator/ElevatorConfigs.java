package frc.robot.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ElevatorConfigs {
    //Comments will be placed in places where we have to tune so we dont forgor💀
    public TalonFXConfiguration elevatorMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.defaultStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.defaultSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = ElevatorConstants.kP; 
        config.Slot0.kI = ElevatorConstants.kI; 
        config.Slot0.kD = ElevatorConstants.kD; 
        config.Slot0.kS = ElevatorConstants.kS; 
        config.Slot0.kV = ElevatorConstants.kV; 
        config.Slot0.kA = ElevatorConstants.kA; 
        config.Slot0.kG = ElevatorConstants.kG;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.forwardSoftLimitMeters / ElevatorConstants.metersPerRotation; //how far we can come out
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.reverseSoftLimitMeters / ElevatorConstants.metersPerRotation; //how far we can come int

        config.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.expoKV;
        config.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.expoKA;

        return config;
    }

    public TalonFXConfiguration idleelevatorMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.reducedStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.reducedSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = ElevatorConstants.kP; 
        config.Slot0.kI = ElevatorConstants.kI; 
        config.Slot0.kD = ElevatorConstants.kD; 
        config.Slot0.kS = ElevatorConstants.kS; 
        config.Slot0.kV = ElevatorConstants.kV; 
        config.Slot0.kA = ElevatorConstants.kA; 
        config.Slot0.kG = ElevatorConstants.kG;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.forwardSoftLimitMeters / ElevatorConstants.metersPerRotation; //how far we can come out
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.reverseSoftLimitMeters / ElevatorConstants.metersPerRotation; //how far we can come int

        config.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.expoKV;
        config.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.expoKA;

        return config;
    }
}