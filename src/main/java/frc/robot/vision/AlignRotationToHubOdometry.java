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
import frc.robot.Constants.VisionAlignConstants;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class AlignRotationToHubOdometry extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final LimeLightSubsystem limelight;

  private final DoubleSupplier driverVx;
  private final DoubleSupplier driverVy;

  private final PIDController headingPID;

  final Optional<Alliance> alliance = DriverStation.getAlliance();
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
        VisionAlignConstants.HEADING_kP,
        VisionAlignConstants.HEADING_kI,
        VisionAlignConstants.HEADING_kD);

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Math.toRadians(VisionAlignConstants.HEADING_TOLERANCE_DEG));
    headingPID.setIntegratorRange(VisionAlignConstants.HEADING_I_MIN, VisionAlignConstants.HEADING_I_MAX);

    addRequirements(drivetrain, limelight);
  }
  
  @Override
  public void initialize() {
    headingPID.reset();
  }

  @Override
  public void execute() {
     int tagID = (int) LimelightHelpers.getFiducialID(VisionAlignConstants.LIMELIGHT_NAME);
    
    if(alliance.isPresent() && alliance.get() == Alliance.Blue) {
      estimatePoseMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionAlignConstants.LIMELIGHT_NAME);
    } else {
      estimatePoseMT1 = LimelightHelpers.getBotPoseEstimate_wpiRed(VisionAlignConstants.LIMELIGHT_NAME);
    }

    if (tagID != -1 && VisionAlignConstants.HUB_TAG_IDS.contains(tagID)) {
      // drivetrain.resetOdometry(new Pose2d(estimatePoseMT2.pose.getX(), estimatePoseMT2.pose.getY(), estimateRotMT1));
      drivetrain.addVisionMeasurement(estimatePoseMT1.pose, estimatePoseMT1.timestampSeconds);
    }
    
    Pose2d robotPose = drivetrain.getState().Pose;
    currentHeading = robotPose.getRotation().getRadians();

    final double vx = driverVx.getAsDouble();
    final double vy = driverVy.getAsDouble();

    double distanceX = VisionAlignConstants.hubPosition().getX() - robotPose.getX();
    double distanceY = VisionAlignConstants.hubPosition().getY() - robotPose.getY();

    desiredHeading = Math.atan2(distanceY, distanceX);
    double omega = headingPID.calculate(currentHeading, desiredHeading);
    
    omega = MathUtil.clamp(
      omega,
      -VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC,
      VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC
    );
    
    drivetrain.setControl(
      driveRequest
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withRotationalRate(omega)
    );

    SmartDashboard.putBoolean("TV", LimelightHelpers.getTV(VisionAlignConstants.LIMELIGHT_NAME));
    SmartDashboard.putNumber("OmegaCmd", omega);
    SmartDashboard.putNumber("DesiredDeg", Math.toDegrees(desiredHeading));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomousEnabled() && Math.abs(desiredHeading - currentHeading) <= Math.toRadians(3);
  }
}