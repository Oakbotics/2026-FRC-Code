package frc.robot.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class HopperConfigs {
    //Comments will be placed in places where we have to tune so we dont forgor💀
    public TalonFXConfiguration elevatorMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        config.CurrentLimits.StatorCurrentLimit = 40; //dont forgor💀
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30; //dont forgor💀
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kP = HopperConstants.kP; //dont forgor💀
        config.Slot0.kI = HopperConstants.kI; //dont forgor💀
        config.Slot0.kD = HopperConstants.kD; //dont forgor💀
        config.Slot0.kS = HopperConstants.kS; //dont forgor💀
        config.Slot0.kV = HopperConstants.kV; //dont forgor💀
        config.Slot0.kA = HopperConstants.kA; //dont forgor💀
        config.Slot0.kG = HopperConstants.kG;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HopperConstants.forwardSoftLimitMeters / HopperConstants.metersPerRotation; //how far we can come out
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HopperConstants.reverseSoftLimitMeters / HopperConstants.metersPerRotation; //how far we can come int

        config.MotionMagic.MotionMagicCruiseVelocity = HopperConstants.cruiseVelocityRPS;
        config.MotionMagic.MotionMagicAcceleration = HopperConstants.accelerationRPSS;

        return config;
    }
}