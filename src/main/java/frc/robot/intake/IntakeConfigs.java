package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConfigs {
    public TalonFXConfiguration intakeMotorConfigs() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        //this set supply limit allows for a 60A burst for 0.1s to overcome inirtia, and then fold back to 30A to protect from brownouts.
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //rises motor from 0 A to commanded amps over 0.25 s for a gentle start which makes all the difference for preventing brownouts and cooking the motor💀
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.25;

        return config;
    }
}
