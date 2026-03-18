package frc.robot.wrist;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class WristConstants {
   // public static final int wristMotorCANId = 3;//Temp
    
    public static final double kP = 90.0; // Also test 250
    public static final double kI = 0.0;
    public static final double kD = 5.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    public static final int gearBoxRatio = 4 * 4 * 4 * (40 / 20);

   /*  public static final double minOutput = -1;
    public static final double maxOutput = 1;
    public static final double velocityFF = 0.5;

    public static final double maxPosition = 225;
    public static final double minPositon = 5;

    // Added feedforward and limits for configs
     public static final double supplyCurrentLimit = 30.0; // TODO: set actual wrist current limit
    public static final double statorCurrentLimit = 30.0; // TODO: set actual wrist stator limit
    */
    
    final double magnetOffset = 0.14;
    final SensorDirectionValue sensorDirectionValue = SensorDirectionValue.CounterClockwise_Positive;
    public final int wristMotorId = 7;
    public final static int wristEncoderId = 37;
    public static final double positionToleranceRotations = 0.01;
  }