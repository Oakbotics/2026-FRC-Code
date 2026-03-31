package frc.robot.hopper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor;
  private final HopperConfigs configs;
  private final MotionMagicVoltage setpointRequest = new MotionMagicVoltage(0);

  public HopperSubsystem() {
    configs = new HopperConfigs();
    elevatorMotor = new TalonFX(HopperConstants.elevatorMotorID);
    //we do this because just in case the configs dont get applied the first time
    for (int i = 0; i < 5; i++) {
      var status = elevatorMotor.getConfigurator().apply(configs.elevatorMotorConfig());
      if (status.isOK()) break;
    }
  }

  public double getPositionMeters() {
    double mechanismRotations = elevatorMotor.getPosition(true).getValueAsDouble();
    return mechanismRotations * HopperConstants.metersPerRotation;
  }

  public void goToPosition(double meters) {
    double targetRotations = meters / HopperConstants.metersPerRotation;
    elevatorMotor.setControl(setpointRequest.withPosition(targetRotations));
  }

  public void zeroHopper() {
    elevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position Meters", getPositionMeters());
  }

  @Override
  public void simulationPeriodic() {
  }
}