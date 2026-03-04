package frc.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConfigs {

        public TalonFXConfiguration shooterMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = ShooterConstants.kP;
            config.Slot0.kI = ShooterConstants.kI;
            config.Slot0.kD = ShooterConstants.kD;
            config.Slot0.kV = ShooterConstants.kV;
            config.Slot0.kS = ShooterConstants.kS;

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