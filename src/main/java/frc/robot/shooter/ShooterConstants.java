package frc.robot.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final double rightKP = 9.0;
    public static final double rightKI = 0.8;
    public static final double rightKD = 0.05;

    public static final double leftKP = 5.75;
    public static final double leftKI = 0.65;
    public static final double leftKD = 0.05;

    public static final double rightKS = 17.5;
    public static final double rightKV = 0.16;

    public static final double leftKS = 17.5;
    public static final double leftKV = 0.04;
    public static final double minOutput = -1;
    public static final double maxOutput = 12;

    // public static final int velocityTorque = 40;

    public static final int leftShooterMotorOneID = 4;
    public static final int leftShooterMotorTwoID = 2;
    public static final int rightShooterMotorOneID = 6;
    public static final int rightShooterMotorTwoID = 5;

    public static final int kickerMotorID = 3;

    public static final double hopperFeedPercent = 0.35;
    public static final double kickerFeedPercent = 0.85;
    public static final double hopperReversePercent = 0.20;
    public static final double kickerReversePercent = 0.45;
    public static final double hopperRecoveryPercent = 0.20;
    public static final double kickerRecorveryPercent = 0.85;

    public static final double hopperJamCurrentAmps = 35.0;

    public static final double kickerJamMinVelocityRPS = 5.0;
    public static final double jamDebounceSec = 0.08;
    public static final double unjamReverseSec = 0.5;
    public static final double unjamRecoverySec = 0.18;


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

    public static final InterpolatingDoubleTreeMap DISTANCE_M_TO_TOF_SEC = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_M_TO_TOF_SEC.put(1.00, 0.35);
        DISTANCE_M_TO_TOF_SEC.put(1.50, 0.40);
        DISTANCE_M_TO_TOF_SEC.put(2.00, 0.46);
        DISTANCE_M_TO_TOF_SEC.put(2.50, 0.52);
        DISTANCE_M_TO_TOF_SEC.put(3.00, 0.58);
        DISTANCE_M_TO_TOF_SEC.put(3.50, 0.64);
        DISTANCE_M_TO_TOF_SEC.put(4.00, 0.70);  
    }

    public static final double MIN_TARGET_RPS = 0.0;
    public static final double MAX_TARGET_RPS = 120.0;

    public static final double VISION_HOLD_LAST_SEC = 0.20;

}