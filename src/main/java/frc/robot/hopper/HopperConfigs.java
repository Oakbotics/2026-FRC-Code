package frc.robot.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConfigs {
    public TalonFXConfiguration hopperMotorConfigs() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;    

        return config;
    }
}
