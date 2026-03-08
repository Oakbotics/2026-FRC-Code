package frc.robot.shooter;

public final class ShootOnMoveConstants {
  private ShootOnMoveConstants() {}

  public static final double PHASE_DELAY_SEC = 0.03;

  public static final double MIN_DISTANCE_M = 0.9;
  public static final double MAX_DISTANCE_M = 4.9;

  public static final double HEADING_kP = 6.0;
  public static final double HEADING_kI = 0.0;
  public static final double HEADING_kD = 0.35;

  public static final double HEADING_I_MIN = -0.25;
  public static final double HEADING_I_MAX = 0.25;

  public static final double HEADING_TOLERANCE_DEG = 2.0;
  public static final double MAX_OMEGA_RAD_PER_SEC = 6.0;

  public static final double LOST_TARGET_GRACE_SEC = 0.20;
}