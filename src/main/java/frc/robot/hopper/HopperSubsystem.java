package frc.robot.hopper;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

  private final TalonFX hopperMotor;
  private final HopperConfigs configs;
  private final VoltageOut voltageOut = new VoltageOut(0); 
  private final PositionVoltage setpointRequest = new PositionVoltage(0);

  public HopperSubsystem() {
    configs = new HopperConfigs();
    hopperMotor = new TalonFX(HopperConstants.hopperMotorID);
    //we do this because just in case the configs dont get applied the first time
    for (int i = 0; i < 5; i++) {
      var status = hopperMotor.getConfigurator().apply(configs.hopperMotorConfig());
      if (status.isOK()) break;
    }
  }

  public void runHopperFront(double speed){
    hopperMotor.setControl(voltageOut.withOutput(speed));
  }

  public void runHopperBack(double speed){
    hopperMotor.setControl(voltageOut.withOutput(-speed));
  }

  public double getPositionMeters() {
    double mechanismRotations = hopperMotor.getPosition(true).getValueAsDouble();
    return mechanismRotations * HopperConstants.metersPerRotation;
  }

  public void goToPosition(double meters) {
    goToPosition(meters, HopperConstants.KVelocity);
  }

  // expoKV controls peak speed: lower = faster (peak vel ≈ 12V / expoKV).
  public void goToPosition(double meters, double expoKV) {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicExpo_kV = expoKV;
    mmConfigs.MotionMagicExpo_kA = HopperConstants.KAcceleration;
    hopperMotor.getConfigurator().apply(mmConfigs);
    double targetRotations = meters / HopperConstants.metersPerRotation;
    hopperMotor.setControl(setpointRequest.withPosition(targetRotations));
  }

  public void zeroHopper() {
    hopperMotor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hopper Position Meters", getPositionMeters());
  }

  @Override
  public void simulationPeriodic() {
  }
}