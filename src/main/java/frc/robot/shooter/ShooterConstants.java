package frc.robot.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final double kP = 95;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double minOutput = -1;
    public static final double maxOutput = 12;

    public static final int shooterMotorOneID = 3;
    public static final int shooterMotorTwoID = 4;
    public static final int shooterMotorThreeID = 5;
    public static final int shooterMotorFourID = 6;

    public static final int kickerMotorID = 7;

    public static final InterpolatingDoubleTreeMap DISTANCE_M_TO_RPS = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_M_TO_RPS.put(1.00, 47.0);
        DISTANCE_M_TO_RPS.put(1.50, 55.0);
        DISTANCE_M_TO_RPS.put(2.00, 62.0);
        DISTANCE_M_TO_RPS.put(2.50, 70.0);
        DISTANCE_M_TO_RPS.put(3.00, 78.0);
        DISTANCE_M_TO_RPS.put(3.50, 86.0);
        DISTANCE_M_TO_RPS.put(4.00, 94.0);
    }

    public static final double MIN_TARGET_RPS = 0.0;
    public static final double MAX_TARGET_RPS = 120.0;

    public static final double VISION_HOLD_LAST_SEC = 0.20;
}