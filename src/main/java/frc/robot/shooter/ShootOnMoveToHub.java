package frc.robot.shooter;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.VisionConstants;
import frc.robot.shooter.HubShotCalculator;
import frc.robot.shooter.HubShotCalculator.Solution;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;
import frc.robot.shooter.ShooterConstants;

public class ShootOnMoveToHub extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final LeftShooterSubsystem leftShooter;
  private final RightShooterSubsystem rightShooter;

  private final DoubleSupplier driverVxMps;
  private final DoubleSupplier driverVyMps;

  private final PIDController headingPID =
      new PIDController(
        ShootOnMoveConstants.HEADING_kP,
        ShootOnMoveConstants.HEADING_kI,
        ShootOnMoveConstants.HEADING_kD);

  private final Timer targetGrace = new Timer();

  private final SwerveRequest.FieldCentric driveRequest =
    new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public ShootOnMoveToHub(
      CommandSwerveDrivetrain drivetrain,
      LeftShooterSubsystem leftShooter,
      RightShooterSubsystem rightShooter,
      DoubleSupplier driverVxMps,
      DoubleSupplier driverVyMps) {

    this.drivetrain = drivetrain;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
    this.driverVxMps = driverVxMps;
    this.driverVyMps = driverVyMps;

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.toRadians(ShootOnMoveConstants.HEADING_TOLERANCE_DEG));
    headingPID.setIntegratorRange(
        ShootOnMoveConstants.HEADING_I_MIN, ShootOnMoveConstants.HEADING_I_MAX);

    addRequirements(drivetrain, leftShooter, rightShooter); 
  }

  @Override
  public void initialize() {
    headingPID.reset();
    targetGrace.restart();
    targetGrace.start();
  }

  @Override
  public void execute() {
    final double vx = driverVxMps.getAsDouble();
    final double vy = driverVyMps.getAsDouble();

    Pose2d robotPose = drivetrain.getState().Pose;

    Translation2d fieldVelMps = new Translation2d(vx, vy);

    Solution sol =
      HubShotCalculator.calculate(
        robotPose,
        fieldVelMps,
        ShooterConstants.DISTANCE_M_TO_RPS,
        ShooterConstants.DISTANCE_M_TO_TOF_SEC,
        ShootOnMoveConstants.PHASE_DELAY_SEC,
        ShootOnMoveConstants.MIN_DISTANCE_M,
        ShootOnMoveConstants.MAX_DISTANCE_M,
        ShooterConstants.MIN_TARGET_RPS,
        ShooterConstants.MAX_TARGET_RPS);

    double currentHeadingRad = robotPose.getRotation().getRadians();
    double desiredHeadingRad = sol.desiredHeading().getRadians();

    double omega =
      headingPID.calculate(currentHeadingRad, desiredHeadingRad);

    omega =
      MathUtil.clamp(
        omega,
        -ShootOnMoveConstants.MAX_OMEGA_RAD_PER_SEC,
        ShootOnMoveConstants.MAX_OMEGA_RAD_PER_SEC);

    drivetrain.setControl(driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

    double targetRps = sol.isValid() ? sol.targetRps() : 0.0;

    leftShooter.runVelocityTorqueFOC(targetRps);
    rightShooter.runVelocityTorqueFOC(-targetRps);

    SmartDashboard.putBoolean("valid", sol.isValid());
    SmartDashboard.putNumber("distanceM", sol.lookaheadDistanceM());
    SmartDashboard.putNumber("tofSec", sol.timeOfFlightSec());
    SmartDashboard.putNumber("targetRps", targetRps);

    double errDeg =
      Math.toDegrees(
        Rotation2d.fromRadians(desiredHeadingRad).minus(robotPose.getRotation()).getRadians());
    SmartDashboard.putNumber("headingErrDeg", errDeg);
  }

  @Override
  public void end(boolean interrupted) {
    leftShooter.setVoltage(0);
    rightShooter.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}