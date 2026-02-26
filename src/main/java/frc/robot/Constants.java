package frc.robot;


import java.util.Optional;
import java.util.Set;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public final class Constants {


  public static final class VisionAlignConstants {
    private VisionAlignConstants() {}


    /** Limelight used for aiming */
    public static final String LIMELIGHT_NAME = "limelight-right";


    /**
     * HUB AprilTags per 2026 REBUILT manual.
     * (IDs: 2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27) :contentReference[oaicite:2]{index=2}
     */
    public static final Set<Integer> HUB_TAG_IDS = Set.of(
        2, 3, 4, 5,
        8, 9, 10, 11,
        18, 19, 20, 21,
        24, 25, 26, 27
    );

   public static Translation2d hubPosition() {
      final Optional<Alliance> alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        return new Translation2d(4.625467, 4.034663);
      }
      return new Translation2d(11.915521, 4.034663);
    }

    /** Heading PID (rad, rad/s) */
    public static final double HEADING_kP = 6.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.35;


    /** Limit integrator so you don’t “wind up” while driver is moving */
    public static final double HEADING_I_MIN = -0.25;
    public static final double HEADING_I_MAX = 0.25;


    /** Finish conditions */
    public static final double HEADING_TOLERANCE_DEG = 1.75;
    public static final double HOLD_TIME_SEC = 0.15;


    /** Target loss behavior */
    public static final double LOST_TARGET_GRACE_SEC = 0.20;


    /** Clamp omega so the robot doesn’t snap violently */
    public static final double MAX_OMEGA_RAD_PER_SEC = 6.0;
  }

}