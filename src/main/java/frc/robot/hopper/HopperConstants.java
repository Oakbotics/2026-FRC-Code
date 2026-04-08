package frc.robot.hopper;

public class HopperConstants {

  public static final int elevatorMotorID = 7;

  public static final double kP = 50.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0; //we set 0 because we are horizontal

  public static final double gearRatio = 64 / 14; // 64 tooth gear is meshing with a 14 tooth gear
  //circumfrence
  public static final double sprocketCircumfrence = 0.045466 * Math.PI;
  public static final double metersPerRotation = gearRatio / sprocketCircumfrence;

  public static final double positionToleranceMeters = 0.005; //dont forgor💀

  public static final double forwardSoftLimitMeters =  1.5; //dont forgor💀
  public static final double reverseSoftLimitMeters = 0.0;

  public static final double cruiseVelocityRPS = 5.0;
  public static final double accelerationRPSS = 5.0;

  public static final double fullyExtended = 0.45;
  public static final double fullyRetracted = 0.0;
  public static final double elevatorFeedingRPS= 3.0;


}