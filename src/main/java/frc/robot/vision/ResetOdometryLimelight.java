// RotateToHubAimPointVision.java
package frc.robot.vision;


import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.VisionConstants;

public class ResetOdometryLimelight extends Command {

  private final CommandSwerveDrivetrain drivetrain;

  PoseEstimate estimatePoseMT1;

  public ResetOdometryLimelight(CommandSwerveDrivetrain drivetrain ) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  
  @Override
  public void initialize() {
    int tagID = (int) LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME);
    
    estimatePoseMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_NAME);

    if (tagID != -1 && estimatePoseMT1 != null) {
      drivetrain.resetOdometry(estimatePoseMT1.pose);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}