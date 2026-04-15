package frc.robot.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class HopperConfigs {
    //Comments will be placed in places where we have to tune so we dont forgor💀
    public TalonFXConfiguration hopperMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        config.CurrentLimits.StatorCurrentLimit = HopperConstants.defaultStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = HopperConstants.defaultSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = HopperConstants.kP; 
        config.Slot0.kI = HopperConstants.kI; 
        config.Slot0.kD = HopperConstants.kD; 
        config.Slot0.kS = HopperConstants.kS; 
        config.Slot0.kV = HopperConstants.kV; 
        config.Slot0.kA = HopperConstants.kA; 
        config.Slot0.kG = HopperConstants.kG;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HopperConstants.forwardSoftLimitMeters / HopperConstants.metersPerRotation; //how far we can come out
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HopperConstants.reverseSoftLimitMeters / HopperConstants.metersPerRotation; //how far we can come int

        config.MotionMagic.MotionMagicExpo_kV = HopperConstants.expoKV;
        config.MotionMagic.MotionMagicExpo_kA = HopperConstants.expoKA;

        return config;
    }

    public TalonFXConfiguration idleHopperMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        config.CurrentLimits.StatorCurrentLimit = HopperConstants.reducedStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = HopperConstants.reducedSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = HopperConstants.kP; 
        config.Slot0.kI = HopperConstants.kI; 
        config.Slot0.kD = HopperConstants.kD; 
        config.Slot0.kS = HopperConstants.kS; 
        config.Slot0.kV = HopperConstants.kV; 
        config.Slot0.kA = HopperConstants.kA; 
        config.Slot0.kG = HopperConstants.kG;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HopperConstants.forwardSoftLimitMeters / HopperConstants.metersPerRotation; //how far we can come out
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HopperConstants.reverseSoftLimitMeters / HopperConstants.metersPerRotation; //how far we can come int

        config.MotionMagic.MotionMagicExpo_kV = HopperConstants.expoKV;
        config.MotionMagic.MotionMagicExpo_kA = HopperConstants.expoKA;

        return config;
    }
}