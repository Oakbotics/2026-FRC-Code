// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.VoltageOut;

public class LeftShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorOne;
    private final TalonFX shooterMotorTwo;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;
    private final Slot0Configs slot0configs = new Slot0Configs();

    /** Creates a new ExampleSubsystem. */
  public LeftShooterSubsystem() {

    shooterMotorOne = new TalonFX(ShooterConstants.shooterMotorOneID);
    shooterMotorTwo = new TalonFX(ShooterConstants.shooterMotorTwoID);
      m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null,
      Volts.of(4),
      null, 
      (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) ->{
          shooterMotorOne.setControl(sysIdControl.withOutput(volts.in(Volts)));
          shooterMotorTwo.setControl(sysIdControl.withOutput(volts.in(Volts)));
        },

        log -> {
          log.motor("Left Shooter Motor One")
          .voltage(shooterMotorOne.getMotorVoltage().getValue())
          .angularPosition(shooterMotorOne.getPosition().getValue())
          .angularVelocity(shooterMotorOne.getVelocity().getValue());

          log.motor("Left Shooter Motor Two")
          .voltage(shooterMotorTwo.getMotorVoltage().getValue())
          .angularPosition(shooterMotorTwo.getPosition().getValue())
          .angularVelocity(shooterMotorTwo.getVelocity().getValue());
        },
      this));
    configureMotors();
    SignalLogger.setPath("/home/vuser/logs/");
    
    
  }

  public void configureMotors() {
    slot0configs.kS = 0.4;
    slot0configs.kV = 0.26;
    slot0configs.kA = 0.0;
    slot0configs.kP = 60;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.0;

    shooterMotorOne.getConfigurator().apply(slot0configs);
    shooterMotorTwo.getConfigurator().apply(slot0configs);

    shooterMotorOne.getVelocity().setUpdateFrequency(100);
    shooterMotorTwo.getVelocity().setUpdateFrequency(100);
    shooterMotorOne.getPosition().setUpdateFrequency(100);
    shooterMotorTwo.getPosition().setUpdateFrequency(100);

  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.dynamic(direction);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.quasistatic(direction);
  }

  public void shootFuel(double speed) {
    shooterMotorOne.setControl(dutyCycle.withOutput(speed));
  }

  public void runVelocity(double rps) {
    voltageRequest.Velocity = rps;
    shooterMotorOne.setControl(voltageRequest); 
  }
  
  public void runVelocityTorqueFOC(double rps) {
      double motorRPS = rps; 
      // double kS_Amps = 0.0; 
      // double kV_Amps = 0.0;
      // double feedForwardAmps = (kS_Amps * Math.signum(rps)) + (kV_Amps * rps);
      double actualRPS = shooterMotorOne.getVelocity().refresh().getValueAsDouble();
      // Create velocity control request
      // VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0)
      //         .withVelocity(motorRPS)
      //         .withFeedForwards(feedForwardAmps);
      final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);


      shooterMotorOne.setControl(request.withVelocity(motorRPS).withFeedForward(0));
      shooterMotorTwo.setControl(request.withVelocity(motorRPS).withFeedForward(0));


      SmartDashboard.putNumber("Motor Target RPS", motorRPS);
      SmartDashboard.putNumber("Motor Actual RPS", actualRPS);
  }


  public void setVoltage(double voltage){
    shooterMotorOne.setVoltage(voltage);
    shooterMotorTwo.setVoltage(voltage);
  }
  
  public void setRawVbus(){
    var voltageRequest = new VoltageOut(0);

    shooterMotorOne.setControl(voltageRequest.withOutput(12));

    printRPM();
  }

  public void printVoltageOutput() {
    double motorVoltage = shooterMotorOne.getMotorVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Motor Voltage", motorVoltage);
  }

  public void resetVoltageOutput() {
    SmartDashboard.putNumber("Motor Voltage", 0);
  }

  public void printCurrentLimits() {
    SmartDashboard.putNumber("Shooter Stator Current", shooterMotorOne.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Supply Current", shooterMotorOne.getSupplyCurrent().getValueAsDouble());
  }

  public void printRPM() {
    double motorRPS = shooterMotorOne.getVelocity().getValueAsDouble();
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