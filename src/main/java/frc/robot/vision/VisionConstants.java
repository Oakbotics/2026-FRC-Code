package frc.robot.vision;


import java.util.Optional;
import java.util.Set;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class VisionConstants {

  private VisionConstants() {}

  public static final String LIMELIGHT_NAME = "limelight-right";

  public static final Set<Integer> HUB_TAG_IDS = Set.of(
      2, 3, 4, 5,
      8, 9, 10, 11,
      18, 19, 20, 21,
      24, 25, 26, 27
  );

  public static Translation2d hubPosition(Optional<Alliance> alliance) {

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      SmartDashboard.putString("Alliance", "blue");
      return new Translation2d(4.625467, 4.034663);
    }
    SmartDashboard.putString("Alliance", "red");
    return new Translation2d(11.915521, 4.034663);
  }

  public static final double HEADING_kP = 4.0;
  public static final double HEADING_kI = 0.0;
  public static final double HEADING_kD = 0.0;

  public static final double HEADING_I_MIN = -0.25;
  public static final double HEADING_I_MAX = 0.25;

  public static final double HEADING_TOLERANCE_DEG = 1.75;
  public static final double HOLD_TIME_SEC = 0.15;

  public static final double LOST_TARGET_GRACE_SEC = 0.20;

  public static final double MAX_OMEGA_RAD_PER_SEC = 6.0;
}