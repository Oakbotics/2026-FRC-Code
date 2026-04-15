package frc.robot.hopper;

import java.util.concurrent.locks.Condition;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor;
  private final HopperConfigs configs;
  private final VoltageOut voltageOut = new VoltageOut(0); 
  private final MotionMagicExpoVoltage setpointRequest = new MotionMagicExpoVoltage(0);

  public HopperSubsystem() {
    configs = new HopperConfigs();
    elevatorMotor = new TalonFX(HopperConstants.hopperMotorID);
    //we do this because just in case the configs dont get applied the first time
    for (int i = 0; i < 5; i++) {
      var status = elevatorMotor.getConfigurator().apply(configs.elevatorMotorConfig());
      if (status.isOK()) break;
    }
  }

  public void runHopperFront(double speed){
    elevatorMotor.setControl(voltageOut.withOutput(speed));
  }

  public void runHopperBack(double speed){
    elevatorMotor.setControl(voltageOut.withOutput(-speed));
  }

  public double getPositionMeters() {
    double mechanismRotations = elevatorMotor.getPosition(true).getValueAsDouble();
    return mechanismRotations * HopperConstants.metersPerRotation;
  }

  public void goToPosition(double meters) {
    goToPosition(meters, HopperConstants.expoKV);
  }

  // expoKV controls peak speed: lower = faster (peak vel ≈ 12V / expoKV).
  public void goToPosition(double meters, double expoKV) {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicExpo_kV = expoKV;
    mmConfigs.MotionMagicExpo_kA = HopperConstants.expoKA;
    elevatorMotor.getConfigurator().apply(mmConfigs);
    elevatorMotor.getConfigurator().apply(configs.elevatorMotorConfig());
    double targetRotations = meters / HopperConstants.metersPerRotation;
    elevatorMotor.setControl(setpointRequest.withPosition(targetRotations));
  }

  public void zeroHopper() {
    elevatorMotor.setPosition(0.0);
  }

  public void applyIdleConfigs() {
    elevatorMotor.getConfigurator().apply(configs.idleElevatorMotorConfig());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position Meters", getPositionMeters());
  }

  @Override
  public void simulationPeriodic() {
  }
}