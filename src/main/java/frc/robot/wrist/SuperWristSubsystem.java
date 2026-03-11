// // Copyright (c) FIRST and other WPILib contributors. 
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.wrist;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.controls. PositionVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com. ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class SuperWristSubsystem extends SubsystemBase {
//   private final TalonFX wristMotor;
//   private final CANcoder wristEncoder;
//   private final PositionVoltage positionRequest = new PositionVoltage(0). withSlot(0);
//   private final VoltageOut voltageRequest = new VoltageOut(0);

//   public SuperWristSubsystem() {
//     wristMotor = new TalonFX(WristConstants.wristMotorCANId);
//     wristEncoder = new CANcoder(3);

//     StatusCode status = wristMotor.getConfigurator().apply(WristConfigs.wristConfig);
//     if (! status.isOK()) {
//       System.err.println("Failed to apply wrist motor config: " + status. toString());
//     }

//     double absolutePose = wristEncoder.getAbsolutePosition().getValueAsDouble();
//     double wristDegrees = absolutePose * 360.0;
//     double motorRotations = degreesToRotations(wristDegrees);

//     wristMotor.setPosition(motorRotations);
//   }

//   public void wristRotateToPosition(double positionDegrees) {
//     wristEncoder.setPosition(0);
//     double clamped = Math.max(WristConstants.minPositon, Math.min(WristConstants.maxPosition, positionDegrees));
//     double targetRotations = degreesToRotations(clamped);

//     wristMotor.setControl(positionRequest.withPosition(targetRotations));
//   }


//   public double getWristAngle() {
//     double rotations = wristMotor.getPosition().getValueAsDouble();
//     return rotationsToDegrees(rotations);
//   }


//   public void printWristPosition() {
//     SmartDashboard.putNumber("Wrist Position", getWristAngle());
//     SmartDashboard.putNumber("Wrist Current ", wristMotor.getSupplyCurrent().getValueAsDouble());
//   }


//   public void setWristSpeed(double speed) {
//     wristMotor.setControl(voltageRequest.withOutput(speed * 12.0));
//   }


//   public void stop() {
//     wristMotor.setControl(voltageRequest.withOutput(0));
//   }


//   private static double degreesToRotations(double degrees) {
//     return degrees / 360.0;
//   }


//   private static double rotationsToDegrees(double rotations) {
//     return rotations * 360.0;
//   }

//   @Override
//   public void periodic() {

//   }

//   @Override
//   public void simulationPeriodic() {

//   }
// }