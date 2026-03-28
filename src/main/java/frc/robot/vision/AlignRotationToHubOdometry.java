// RotateToHubAimPointVision.java
package frc.robot.vision;


import java.util.Optional;
import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.VisionConstants;

public class AlignRotationToHubOdometry extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final LimeLightSubsystem limelight;
  // FIX: Removed stale field-level alliance. It was captured at construction
  // time (during robotInit) before the DriverStation knows the real alliance.

  private final DoubleSupplier driverVx;
  private final DoubleSupplier driverVy;

  private final PIDController headingPID;
  
  PoseEstimate estimatePoseMT1;

  double currentHeading;
  double desiredHeading;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignRotationToHubOdometry(CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limelight, DoubleSupplier vx, DoubleSupplier vy) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.driverVx = vx;
    this.driverVy = vy;

    headingPID = new PIDController(
        VisionConstants.HEADING_kP,
        VisionConstants.HEADING_kI,
        VisionConstants.HEADING_kD);

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.toRadians(VisionConstants.HEADING_TOLERANCE_DEG));
    headingPID.setIntegratorRange(VisionConstants.HEADING_I_MIN, VisionConstants.HEADING_I_MAX);

    addRequirements(drivetrain, limelight);
  }
  
  @Override
  public void initialize() {
    headingPID.reset();
  }

  @Override
  public void execute() {

    
    Pose2d robotPose = drivetrain.getState().Pose;
    currentHeading = robotPose.getRotation().getRadians();

    final double vx = driverVx.getAsDouble();
    final double vy = driverVy.getAsDouble();

    double distanceX = VisionConstants.hubPosition().getX() - robotPose.getX();
    double distanceY = VisionConstants.hubPosition().getY() - robotPose.getY();

    desiredHeading = Math.atan2(distanceY, distanceX);
    double omega = headingPID.calculate(currentHeading, desiredHeading);
    
    omega = MathUtil.clamp(
      omega,
      -VisionConstants.MAX_OMEGA_RAD_PER_SEC,
      VisionConstants.MAX_OMEGA_RAD_PER_SEC
    );
    
    drivetrain.setControl(
      driveRequest
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withRotationalRate(omega)
    );

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomousEnabled() && Math.abs(desiredHeading - currentHeading) <= Math.toRadians(3);
  }
}