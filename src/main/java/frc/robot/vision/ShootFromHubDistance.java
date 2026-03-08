package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;
import frc.robot.shooter.ShooterConstants;

public class ShootFromHubDistance extends Command {
  private final LeftShooterSubsystem leftShooter;
  private final RightShooterSubsystem rightShooter;
  private final LimeLightSubsystem limelight;

  private final Timer sinceLastVision = new Timer();
  private double lastTargetRps = 0.0;

  public ShootFromHubDistance(
      LeftShooterSubsystem leftShooter,
      RightShooterSubsystem rightShooter,
      LimeLightSubsystem limelight) {

    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
    this.limelight = limelight;

    addRequirements(leftShooter, rightShooter);
  }

  @Override
  public void initialize() {
    lastTargetRps = 0.0;
    sinceLastVision.restart();
  }

  @Override
  public void execute() {
    boolean hasHubTarget = limelight.hasHubTarget();

    double distanceM = Double.NaN;
    double targetRps = lastTargetRps;

    if (hasHubTarget) {
      sinceLastVision.reset();
      sinceLastVision.start();

      distanceM = limelight.getDistanceToHubMeters();

      double lookupRps = ShooterConstants.DISTANCE_M_TO_RPS.get(distanceM);
      targetRps = MathUtil.clamp(lookupRps, ShooterConstants.MIN_TARGET_RPS, ShooterConstants.MAX_TARGET_RPS);

      lastTargetRps = targetRps;
    } else {
      if (sinceLastVision.get() > ShooterConstants.VISION_HOLD_LAST_SEC) {
        targetRps = 0.0;
        lastTargetRps = 0.0;
      }
    }

    leftShooter.runVelocityTorqueFOC(targetRps);
    rightShooter.runVelocityTorqueFOC(targetRps);

    SmartDashboard.putBoolean("HasHubTarget", hasHubTarget);
    SmartDashboard.putNumber("DistanceToHub_m", distanceM);
    SmartDashboard.putNumber("TargetRPS", targetRps);
  }

  @Override
  public void end(boolean interrupted) {
    leftShooter.runVelocityTorqueFOC(0.0);
    rightShooter.runVelocityTorqueFOC(0.0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}