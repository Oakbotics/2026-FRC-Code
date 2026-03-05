package frc.robot.wrist;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class WristConstants {
    public static final int wristMotorCANId = 3;//Temp
    
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double kV = 0.0; // TODO: tune feedforward V
    public static final double kS = 0.0; // TODO: tune feedforward S

    public static final int gearBoxRatio = 2;

   /*  public static final double minOutput = -1;
    public static final double maxOutput = 1;
    public static final double velocityFF = 0.5;

    public static final double maxPosition = 225;
    public static final double minPositon = 5;

    // Added feedforward and limits for configs
     public static final double supplyCurrentLimit = 30.0; // TODO: set actual wrist current limit
    public static final double statorCurrentLimit = 30.0; // TODO: set actual wrist stator limit
    */
    
    final double magnetOffset = 0.0;
    final SensorDirectionValue SensorDirectionValue = null;
    public final int wristMotorId = 3;
    public final int wristEncoderId = 4;
  }