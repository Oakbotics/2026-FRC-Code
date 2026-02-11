// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import javax.xml.validation.SchemaFactoryConfigurationError;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.Configs;
import frc.robot.Configs.ShooterConfigs;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import com.ctre.phoenix6.controls.VoltageOut;

public class RightShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorThree;
    private final TalonFX shooterMotorFour;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;

    /** Creates a new ExampleSubsystem. */
  public RightShooterSubsystem() {
    shooterMotorThree = new TalonFX(ShooterConstants.shooterMotorThreeID);
    shooterMotorFour = new TalonFX(ShooterConstants.shooterMotorFourID);
      m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null,
      Volts.of(4),
      null, 
      (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) -> shooterMotorThree.setControl(sysIdControl.withOutput(volts.in(Volts))),
        log -> {
          log.motor("Shooter Motor")
          .voltage(shooterMotorThree.getMotorVoltage().getValue())
          .angularPosition(shooterMotorThree.getPosition().getValue())
          .angularVelocity(shooterMotorThree.getVelocity().getValue());
        },
      this));
    configureMotors();
    SignalLogger.setPath("/home/vuser/logs/");
    
  }

  public void configureMotors() {
    shooterMotorThree.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());
    shooterMotorFour.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());

    shooterMotorThree.getVelocity().setUpdateFrequency(100);
    shooterMotorThree.getPosition().setUpdateFrequency(100);

  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.dynamic(direction);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.quasistatic(direction);
  }

  public void shootFuel(double speed) {
    shooterMotorThree.setControl(dutyCycle.withOutput(speed));
  }

  public void runVelocity(double rps) {
    voltageRequest.Velocity = rps;
    shooterMotorThree.setControl(voltageRequest); 
  }
  
public void runVelocityTorqueFOC(double rps) {
    double motorRPS = rps; // gear ratio included if needed
    double kS_Amps = 0.2; 
    double kV_Amps = 0.1;
    double feedForwardAmps = (kS_Amps * Math.signum(rps)) + (kV_Amps * rps);
    // Create velocity control request
    VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0)
            .withVelocity(motorRPS)
            .withFeedForward(feedForwardAmps); // small feedforward to help startup

    // Send control to motor
    shooterMotorThree.setControl(request);
    shooterMotorFour.setControl(request);

    // Logging
    SmartDashboard.putNumber("Motor Target RPS", motorRPS);
    SmartDashboard.putNumber("Motor Actual RPS", shooterMotorThree.getVelocity().getValueAsDouble());
}



  

  public void printVoltageOutput() {
    double motorVoltage = shooterMotorThree.getMotorVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Motor Voltage", motorVoltage);
  }

  public void resetVoltageOutput() {
    SmartDashboard.putNumber("Motor Voltage", 0);
  }

  public void printCurrentLimits() {
    SmartDashboard.putNumber("Shooter Stator Current", shooterMotorThree.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Supply Current", shooterMotorThree.getSupplyCurrent().getValueAsDouble());
  }

  public void printRPM() {
    double motorRPS = shooterMotorThree.getVelocity().getValueAsDouble();
    double shooterRPM = motorRPS * 60.0;
    SmartDashboard.putNumber("Shooter Motor RPM", shooterRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}