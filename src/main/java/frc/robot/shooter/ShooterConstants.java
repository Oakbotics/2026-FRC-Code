package frc.robot.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static double rightKP = 8.5;
    public static double rightKI = 0.0;
    public static double rightKD = 0.0;

    public static double leftKP = 8.0;
    public static double leftKI = 0.0;
    public static double leftKD = 0.0;

    public static double rightKS = 17.5;
    public static double rightKV = 0.013;

    public static double leftKS = 17.5;
    public static double leftKV = 0.013;

    public static double kickerKP = 5.75;
    public static double kickerKI = 0.65;
    public static double kickerKD = 0.05;

    public static double kickerKS = 17.5;
    public static double kickerKV = 0.16;
    
    public static final double minOutput = -1;
    public static final double maxOutput = 12;

    // public static final int velocityTorque = 40;

    public static final int leftShooterMotorOneID = 4;
    public static final int leftShooterMotorTwoID = 2;
    public static final int rightShooterMotorOneID = 6;
    public static final int rightShooterMotorTwoID = 5;

    public static final int kickerMotorID = 3;

    public static final double hopperFeedPercent = 1.0;
    public static final double kickerFeedPercent = 8.0;
    public static final double hopperReversePercent = 0.20;
    public static final double kickerReversePercent = 8.0;
    public static final double hopperRecoveryPercent = 0.20;
    public static final double kickerRecorveryPercent = 8.0;

    public static final double hopperJamCurrentAmps = 35.0;

    public static final double kickerJamMinVelocityRPS = 5.0;
    public static final double jamDebounceSec = 0.08;
    public static final double unjamReverseSec = 0.5;
    public static final double unjamRecoverySec = 0.18;

    public static final InterpolatingDoubleTreeMap DISTANCE_M_TO_RPS = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_M_TO_RPS.put(2.36, 50.75);
        DISTANCE_M_TO_RPS.put(2.53, 51.375);
        DISTANCE_M_TO_RPS.put(2.7, 52.0);
        DISTANCE_M_TO_RPS.put(2.875, 53.5);
        DISTANCE_M_TO_RPS.put(3.05, 55.0);
        DISTANCE_M_TO_RPS.put(3.145, 55.7);
        DISTANCE_M_TO_RPS.put(3.24, 56.4);
        DISTANCE_M_TO_RPS.put(3.47, 57.45);
        DISTANCE_M_TO_RPS.put(3.70, 58.5);
        DISTANCE_M_TO_RPS.put(3.865, 60.25);
        DISTANCE_M_TO_RPS.put(4.03, 62.0);
        DISTANCE_M_TO_RPS.put(4.95, 67.25);
    }

    public static final InterpolatingDoubleTreeMap DISTANCE_M_TO_TOF_SEC = new InterpolatingDoubleTreeMap();

    
    static {
        DISTANCE_M_TO_TOF_SEC.put(2.36, 0.35);
        DISTANCE_M_TO_TOF_SEC.put(2.53, 0.40);
        DISTANCE_M_TO_TOF_SEC.put(2.7, 0.46);
        DISTANCE_M_TO_TOF_SEC.put(2.875, 0.52);
        DISTANCE_M_TO_TOF_SEC.put(3.05, 0.58);
        DISTANCE_M_TO_TOF_SEC.put(3.145, 0.64);
        DISTANCE_M_TO_TOF_SEC.put(3.24, 0.70);
        // DISTANCE_M_TO_RPS.put(3.47, 0.76);
        // DISTANCE_M_TO_RPS.put(3.70, 0.8);
        // DISTANCE_M_TO_RPS.put(3.865, 0.82);
        // DISTANCE_M_TO_RPS.put(4.03, 0.87);
        // DISTANCE_M_TO_RPS.put(4.95, 1.2);
    }

    public static final double MIN_TARGET_RPS = 0.0;
    public static final double MAX_TARGET_RPS = 100.0;

    public static final double VISION_HOLD_LAST_SEC = 0.20;

}