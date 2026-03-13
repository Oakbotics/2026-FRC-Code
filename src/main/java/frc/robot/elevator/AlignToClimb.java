package frc.robot.elevator;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.VisionConstants;

public class AlignToClimb extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  private final PIDController xPID = new PIDController(2.5, 0.0, 0.0);
  private final PIDController yPID = new PIDController(2.5, 0.0, 0.0);
  private final PIDController headingPID;

  private Pose2d climbPosition;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToClimb(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    headingPID = new PIDController(
        VisionConstants.HEADING_kP,
        VisionConstants.HEADING_kI,
        VisionConstants.HEADING_kD);

    headingPID.enableContinuousInput(-Math.PI, Math.PI);

    xPID.setTolerance(0.01);
    yPID.setTolerance(0.01);
    headingPID.setTolerance(Math.toRadians(1));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {

    Optional<Alliance> alliance = DriverStation.getAlliance();

    // Climb position left/right is from the POV of the driver station
    // Climb coordinates need to be changed at competition

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      if (drivetrain.getState().Pose.getY() > 3.7) {
        climbPosition = new Pose2d(1.064, 4.672, Rotation2d.fromDegrees(90));
      } else {
        climbPosition = new Pose2d(1.064, 2.831, Rotation2d.fromDegrees(-90));
      }
    } else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      if (drivetrain.getState().Pose.getY() > 4.3) {
        climbPosition = new Pose2d(15.491, 5.235, Rotation2d.fromDegrees(90));
      } else {
        climbPosition = new Pose2d(15.491, 3.415, Rotation2d.fromDegrees(-90));
      }
    }

    xPID.reset();
    yPID.reset();
    headingPID.reset();
  }

  @Override
  public void execute() {

    Pose2d robotPose = drivetrain.getState().Pose;

    double vx = xPID.calculate(robotPose.getX(), climbPosition.getX());
    double vy = yPID.calculate(robotPose.getY(), climbPosition.getY());

    double omega = headingPID.calculate(
        robotPose.getRotation().getRadians(),
        climbPosition.getRotation().getRadians());

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

