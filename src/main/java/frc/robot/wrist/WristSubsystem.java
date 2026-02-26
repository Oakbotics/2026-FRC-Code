// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;



public class WristSubsystem extends SubsystemBase {
    
  private final TalonFX wristMotor;

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private final VoltageOut voltageRequest = new VoltageOut(0);


  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristMotor = new TalonFX(WristConstants.wristMotorCANId);

    wristMotor.setPosition(0);
    StatusCode status = wristMotor.getConfigurator().apply(Configs.WristConfigs.wristConfig);
  
  }

  public void wristRotateToPosition(double positionDegrees){
    double targetRotations = degreesToRotations(positionDegrees);
    wristMotor.setControl(positionRequest.withPosition(targetRotations));
  }

  public double getWristAngle(){
    double rotations = wristMotor.getPosition().getValueAsDouble();
    return rotationsToDegrees(rotations);
  }

  public double getWristVelocity(){
    double rotationsPerSecond = wristMotor.getVelocity().getValueAsDouble();
    return rotationsPerSecond * 360.0;
  }

  public void stop(){
    wristMotor.setControl(voltageRequest.withOutput(0));
  }


  public void printWristPosition(){
    SmartDashboard.putNumber("Wrist Position", getWristAngle());
    SmartDashboard.putNumber("Wrist Velocity (deg/second)", getWristVelocity());
  }




  private static double degreesToRotations(double degrees){

    return degrees / 360;
  }

  private static double rotationsToDegrees(double rotations){

    return rotations * 360;
  }


//   public Angle setPosition(Supplier<Angle> angle) {
//     return run(() -> {
        
//     });
//   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}