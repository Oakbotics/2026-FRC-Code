package frc.robot.ShootOnTheMove;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class HubShotCalculator {

  public record Solution(
      boolean isValid,
      Rotation2d desiredHeading,
      double lookaheadDistanceM,
      double timeOfFlightSec,
      double targetRps) {}

  private HubShotCalculator() {}

  public static Solution calculate(
      Pose2d robotPose,
      Translation2d fieldVelMps,
      Translation2d hubPos,
      Transform2d robotToShooter,
      InterpolatingDoubleTreeMap distanceToRps,
      InterpolatingDoubleTreeMap distanceToTofSec,
      double phaseDelaySec,
      double minDistanceM,
      double maxDistanceM,
      double minRps,
      double maxRps) {

    if (robotPose == null || fieldVelMps == null || hubPos == null) {
      return new Solution(false, Rotation2d.kZero, Double.NaN, Double.NaN, 0.0);
    }

    Translation2d projectedRobotTranslation =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(
                    fieldVelMps.getX() * phaseDelaySec, fieldVelMps.getY() * phaseDelaySec));
    Pose2d projectedRobotPose = new Pose2d(projectedRobotTranslation, robotPose.getRotation());

    Translation2d shooterPos = projectedRobotPose.transformBy(robotToShooter).getTranslation();

    Translation2d lookaheadShooterPos = shooterPos;
    double dist = lookaheadShooterPos.getDistance(hubPos);
    double tof = distanceToTofSec.get(dist);

    for (int i = 0; i < 8; i++) {
      tof = distanceToTofSec.get(dist);
      lookaheadShooterPos =
          shooterPos.plus(
              new Translation2d(fieldVelMps.getX() * tof, fieldVelMps.getY() * tof));
      dist = lookaheadShooterPos.getDistance(hubPos);
    }

    Rotation2d desiredHeading = hubPos.minus(lookaheadShooterPos).getAngle();

    double rps = distanceToRps.get(dist);
    rps = MathUtil.clamp(rps, minRps, maxRps);

    boolean valid = Double.isFinite(dist) && dist >= minDistanceM && dist <= maxDistanceM;

    return new Solution(valid, desiredHeading, dist, tof, rps);
  }
}