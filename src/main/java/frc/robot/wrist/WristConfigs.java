package frc.robot.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.shooter.ShooterConstants;
import frc.robot.wrist.WristConstants;

public final class WristConfigs {
        public static final TalonFXConfiguration wristConfig = new TalonFXConfiguration();

        public static final double WRIST_GEAR_RATIO = 1.0;
        
        static {
    
            wristConfig. MotorOutput.Inverted = InvertedValue. Clockwise_Positive;

            wristConfig. MotorOutput.NeutralMode = NeutralModeValue.Brake;

            wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            wristConfig. CurrentLimits. SupplyCurrentLimit = WristConstants.supplyCurrentLimit;

            wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            wristConfig.CurrentLimits.StatorCurrentLimit = WristConstants.statorCurrentLimit;


            wristConfig.Feedback. SensorToMechanismRatio = WRIST_GEAR_RATIO;

            wristConfig.Slot0.kP = WristConstants.kP;
            wristConfig.Slot0.kI = WristConstants.kI;
            wristConfig.Slot0.kD = WristConstants.kD;
            wristConfig.Slot0.kV = WristConstants. kV;
            wristConfig. Slot0.kS = WristConstants. kS;

            wristConfig. SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            wristConfig.SoftwareLimitSwitch. ForwardSoftLimitThreshold = WristConstants.maxPosition / 360.0;
            wristConfig. SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            wristConfig. SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.minPositon / 360.0;

            wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    }
}