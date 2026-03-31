package frc.robot.drive;

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
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.vision.VisionConstants;

public class AlignToTrench extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private final PIDController xPID = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yPID = new PIDController(5.0, 0.0, 0.0);
  private final PIDController headingPID;

  private Pose2d blueLeftTrenchPosition = new Pose2d(0.0, 7.436, Rotation2d.fromDegrees(0));
  private Pose2d blueRightTrenchPosition = new Pose2d(0, 0.645, Rotation2d.fromDegrees(0));
  private Pose2d redLeftTrenchPosition = new Pose2d(0, 0.645, Rotation2d.fromDegrees(180));
  private Pose2d redRightTrenchPosition = new Pose2d(0, 7.436, Rotation2d.fromDegrees(180));

  private Pose2d trenchPos;
  Optional<Alliance> alliance;
  private DoubleSupplier driverVx;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToTrench(CommandSwerveDrivetrain drivetrain, DoubleSupplier vx) {
    this.drivetrain = drivetrain;

    headingPID = new PIDController(
        VisionConstants.HEADING_kP,
        VisionConstants.HEADING_kI,
        VisionConstants.HEADING_kD);

    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    this.driverVx = vx;

    xPID.setTolerance(0.01);
    yPID.setTolerance(0.01);
    headingPID.setTolerance(Math.toRadians(1));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance();

    xPID.reset();
    yPID.reset();
    headingPID.reset();
  }

  @Override
  public void execute() {

    Pose2d robotPose = drivetrain.getState().Pose;

    if (alliance.get() == Alliance.Blue && robotPose.getY() >= 4.0) {
        trenchPos = blueLeftTrenchPosition;
    } else if (alliance.get() == Alliance.Blue && robotPose.getY() < 4.0) {
        trenchPos = blueRightTrenchPosition;
    } else if (alliance.get() == Alliance.Red && robotPose.getY() < 4.0) {
        trenchPos = redLeftTrenchPosition;
    } else if (alliance.get() == Alliance.Red && robotPose.getY() >= 4.0) {
        trenchPos = redRightTrenchPosition;
    }

    double vx =  driverVx.getAsDouble();
    double vy = yPID.calculate(robotPose.getY(), trenchPos.getY());

    double omega = headingPID.calculate(
        robotPose.getRotation().getRadians(),
        trenchPos.getRotation().getRadians());

    vx = MathUtil.clamp(vx, -2.0, 2.0);
    vy = MathUtil.clamp(vy, -2.0, 2.0);

    omega = MathUtil.clamp(
        omega,
        -VisionConstants.MAX_OMEGA_RAD_PER_SEC,
        VisionConstants.MAX_OMEGA_RAD_PER_SEC);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omega));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}