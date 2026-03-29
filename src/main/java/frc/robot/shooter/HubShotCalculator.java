package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.vision.VisionConstants;

public final class HubShotCalculator {

  public record Solution(
      boolean isValid,
      Rotation2d desiredHeading,
      double lookaheadDistanceM,
      double timeOfFlightSec,
      double targetRps) {}

  private HubShotCalculator() {}

  public static Solution calculate (
    Pose2d robotPose,
    Translation2d fieldVelMps,
    InterpolatingDoubleTreeMap distanceToRps,
    InterpolatingDoubleTreeMap distanceToTOFSec,
    double phaseDelaySec,
    double minDistanceM,
    double maxDistanceM,
    double minRps,
    double maxRps) {

    if (robotPose == null || fieldVelMps == null) {
      return new Solution(false, Rotation2d.kZero, Double.NaN, Double.NaN, 0.0);
    }

    Translation2d basePos =
    robotPose.getTranslation().plus(
      new Translation2d(
        fieldVelMps.getX() * phaseDelaySec,
        fieldVelMps.getY() * phaseDelaySec));

    Translation2d lookaheadPos = basePos;
    
    double dist = lookaheadPos.getDistance(VisionConstants.hubPosition(DriverStation.getAlliance()));
    double tof = distanceToTOFSec.get(dist);

    for (int i = 0; i < 8; i++) {

      tof = distanceToTOFSec.get(dist);

      lookaheadPos =
        basePos.plus(
          new Translation2d(
            fieldVelMps.getX() * tof,
            fieldVelMps.getY() * tof));

      dist = lookaheadPos.getDistance(VisionConstants.hubPosition(DriverStation.getAlliance()));
    }

  Rotation2d desiredHeading =
      VisionConstants.hubPosition(DriverStation.getAlliance()).minus(lookaheadPos).getAngle();

    double rps = distanceToRps.get(dist);
    rps = MathUtil.clamp(rps, minRps, maxRps);

    boolean valid = Double.isFinite(dist) && dist >= minDistanceM && dist <= maxDistanceM;

    return new Solution(valid, desiredHeading, dist, tof, rps);
  }
}