package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristEncoderSubsystem extends SubsystemBase {

    
    private static final int ENCODER_CAN_ID = 10;

    // Replace after zeroing mechanism
    private static final double MAGNET_OFFSET = 0.0;

    private static final boolean SENSOR_INVERTED = false;

    public Angle initialWristAngle;

    private final CANcoder encoder;

    public AbsoluteEncoderSubsystem() {

        encoder = new CANcoder(ENCODER_CAN_ID);

        CANcoderConfiguration config = new CANcoderConfiguration();

        

        config.MagnetSensor.AbsoluteSensorRange =
                AbsoluteSensorRangeValue.Unsigned_0To1;

        config.MagnetSensor.SensorDirection = SENSOR_INVERTED;

        config.MagnetSensor.MagnetOffset = MAGNET_OFFSET;

        encoder.getConfigurator().apply(config);

        initialWristAngle = encoder.getAbsolutePosition();
    }

    /** Returns raw absolute position (0.0 - 1.0) */
    public Angle getRawPosition() {
        return encoder.getAbsolutePosition();
    }
    public Angle getInitialPosition(){
      return initialWristAngle;
    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Wrist encoder position", getRawPosition());

    }
}