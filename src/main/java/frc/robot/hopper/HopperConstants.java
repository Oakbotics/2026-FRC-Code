package frc.robot.hopper;

public class HopperConstants {

  public static final int hopperMotorID = 7;

  public static final double kP = 10.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 2.0;
  public static final double kV = 0.64;
  public static final double kA = 0.01;
  public static final double kG = 0.5; //we set 0 because we are horizontal

  public static final double gearRatio = 64.0 / 14.0; // 64 tooth gear is meshing with a 14 tooth gear
  //circumfrence
  public static final double sprocketCircumfrence = 0.045466 * Math.PI;
  public static final double metersPerRotation = sprocketCircumfrence / gearRatio;

  public static final double positionToleranceMeters = 0.0;

  public static final double forwardSoftLimitMeters = 0.4;
  public static final double reverseSoftLimitMeters = 0.0;

  public static final double KVelocity = 0.5;

  public static final double KAcceleration = 1.0;

  public static final double fullyExtended = 0.3;
  public static final double fullyRetracted = 0.15;
  public static final double hopperFeedingRPS = 3.0;
}