package frc.robot.elevator;

public class ElevatorConstants {
  
  public static final int elevatorMotorID = 7;

  public static final double kP = 5.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 1.0;
  public static final double kV = 0.13;
  public static final double kA = 0.01;
  public static final double kG = 1.0; //we set 0 because we are horizontal

  public static final double gearRatio = 4 * 5 ; // 4:1 gearbox multiplied by 5:1 gearbox
  //circumfrence
  public static final double pulleyCircumfrence = 0.047752 * Math.PI;
  public static final double metersPerRotation = pulleyCircumfrence / gearRatio;

  public static final double positionToleranceMeters = 0.05;

  public static final double forwardSoftLimitMeters = 0.4;
  public static final double reverseSoftLimitMeters = 0.0;

  public static final double defaultStatorCurrentLimit = 50.0;
  public static final double defaultSupplyCurrentLimit = 40.0;

  public static final double reducedStatorCurrentLimit = 20.0;
  public static final double reducedSupplyCurrentLimit = 15.0;

  //   Peak velocity ≈ 12V / expoKV. At 0.12 → ~100 rot/s motor speed.
  public static final double expoKV = 0.12;

  // volts per (motor rot/s²). Controls how hard the motor pushes during acceleration.
  public static final double expoKA = 0.10;
  public static final double cruiseVelocityRPS = 20.0;
  public static final double accelerationRPSS = 40.0;

  public static final double fullyExtended = 0.3;
  public static final double fullyRetracted = 0.04;
  public static final double elevatorFeedingRPS = 3.0;

  public static final double elevatorAgitateDelay = 0.15;
}