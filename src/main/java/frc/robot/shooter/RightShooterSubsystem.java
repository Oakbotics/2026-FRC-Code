// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.VoltageOut;

public class RightShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorThree;
    private final TalonFX shooterMotorFour;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;
    private final ShooterConfigs configs;

    /** Creates a new ExampleSubsystem. */
  public RightShooterSubsystem() {

    shooterMotorThree = new TalonFX(ShooterConstants.rightShooterMotorOneID);
    shooterMotorFour = new TalonFX(ShooterConstants.rightShooterMotorTwoID);
    configs = new ShooterConfigs();
      m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null,
      Volts.of(4),
      null, 
      (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) -> {
          shooterMotorThree.setControl(sysIdControl.withOutput(volts.in(Volts)));
          shooterMotorFour.setControl(sysIdControl.withOutput(volts.in(Volts)));
        },

        log -> {
          log.motor("Right Shooter Motor One")
          .voltage(shooterMotorThree.getMotorVoltage().getValue())
          .angularPosition(shooterMotorThree.getPosition().getValue())
          .angularVelocity(shooterMotorThree.getVelocity().getValue());

          log.motor("Right Shooter Motor Two")
          .voltage(shooterMotorFour.getMotorVoltage().getValue())
          .angularPosition(shooterMotorFour.getPosition().getValue())
          .angularVelocity(shooterMotorFour.getVelocity().getValue());
        },
      this));
    configureMotors();
    SignalLogger.setPath("/home/vuser/logs/");
    
  }
  // 44 inch from hub
  public void configureMotors() {

    for (int i = 0; i < 5; i++) {
      var status = shooterMotorThree.getConfigurator().apply(configs.shooterMotorConfig());
      if (status.isOK()) break;
    }

    for (int i = 0; i < 5; i++) {
      var status = shooterMotorFour.getConfigurator().apply(configs.shooterMotorConfig());
      if (status.isOK()) break;
    }

    shooterMotorThree.getVelocity().setUpdateFrequency(100);
    shooterMotorFour.getVelocity().setUpdateFrequency(100);
    shooterMotorThree.getPosition().setUpdateFrequency(100);
    shooterMotorFour.getPosition().setUpdateFrequency(100);

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
  
  private final VelocityTorqueCurrentFOC torqueFocRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);  

  public void runVelocityTorqueFOC(double rps) {
      double motorRPS = -rps; 
      double actualRPS = shooterMotorThree.getVelocity().refresh().getValueAsDouble();


      shooterMotorThree.setControl(torqueFocRequest.withVelocity(rps));
      shooterMotorFour.setControl(torqueFocRequest.withVelocity(rps));

      
      SmartDashboard.putNumber("RIGHT Motor Target RPS", motorRPS);
      SmartDashboard.putNumber("RIGHT Motor Actual RPS", actualRPS);
  }


  public void setVoltage(double voltage) {
    shooterMotorThree.setVoltage(voltage);
    shooterMotorFour.setVoltage(voltage);
  }
  
  public void setRawVbus() {
    var voltageRequest = new VoltageOut(0);

    shooterMotorThree.setControl(voltageRequest.withOutput(12));

    printRPM();
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