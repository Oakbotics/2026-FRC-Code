package frc.robot.hopper;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class HopperConstants {
  public final int hopperMotorId = 7;  
  
  public static final double kP = 155.0; // Also test 250
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;
  public static final double kS = 0.5;
  public static final double kV = 0.6;

  public static final int gearBoxRatio = 4 * 4 * 4 * (40 / 20);
  public static final double metersPerRotation = 3739;

  public static final Angle angleDown = Degrees.of(121);
  public static final Angle angleUp = Degrees.of(74);

  public static final double positionToleranceMeters = 3;
}