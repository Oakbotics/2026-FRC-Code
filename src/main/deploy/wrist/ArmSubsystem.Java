import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmSubsystem {
    // 1. Declare Motor
    private TalonFX armMotor = new TalonFX(3); // Replace 0 with your CAN ID

    // 2. Control Request
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // Gear ratio: Rotor Rotations to Mechanism Rotations
    private final double kGearRatio = 48.0; 

    public ArmSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 3. Configure Slot 0 (PID)
        configs.Slot0.kP = 10.0; // Tune these
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.1;
        configs.Slot0.kV = 0.12; // Velocity feedforward
        configs.Slot0.kA = 0.01; // Acceleration feedforward
        configs.Slot0.kG = 0.25; // Gravity feedforward (units in rotations)
        configs.Slot0.GravityType = GravityTypeValue.Cosine; // Cosine for arms

        // 4. Configure Motion Magic
        configs.MotionMagic.MotionMagicCruiseVelocity = 1.0; // Rotations/sec
        configs.MotionMagic.MotionMagicAcceleration = 0.5; // Rotations/sec^2
        configs.MotionMagic.MotionMagicJerk = 20.0; // Rotations/sec^3

        // 5. Motor Output & Feedback
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Feedback.SensorToMechanismRatio = kGearRatio;

        armMotor.getConfigurator().apply(configs);
    }

    // Command to set position in rotations
    public void setPosition(double rotations) {
        armMotor.setControl(m_request.withPosition(rotations));
    }
}
