package frc.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RightShooterConfigs {
    public TalonFXConfiguration shooterMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.rightKP;
        config.Slot0.kI = ShooterConstants.rightKI;
        config.Slot0.kD = ShooterConstants.rightKD;
        config.Slot0.kV = ShooterConstants.rightKV;
        config.Slot0.kS = ShooterConstants.rightKS;

        config.Feedback.SensorToMechanismRatio = 1.0;
        
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;    

        return config;
    }
}