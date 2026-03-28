// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.vision;

import java.lang.StackWalker.Option;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;


public class LimeLightSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain drivetrain;
  public NetworkTable m_limeLightRightTable;
  public NetworkTable m_limeLightTopTable;
  public NetworkTable m_limeLightLeftTable;
  public Pose2d m_closestTagPose;
  // Limelight Left: http://10.37.39.11:5801/
  //Limelight Right: http://10.37.39.12:5801/
 
  /** Creates a new LimeLightSubsystem. */
  public LimeLightSubsystem(CommandSwerveDrivetrain drivetrain) {
    m_limeLightRightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
    m_limeLightTopTable = NetworkTableInstance.getDefault().getTable("limelight-top");
    m_limeLightLeftTable = NetworkTableInstance.getDefault().getTable("limelight-left");
    m_limeLightRightTable.getEntry("pipeline").setNumber(0);
    // SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("Shooter RPS", 58.5);

    this.drivetrain = drivetrain;
  }

  public Pose2d getBotPoseRightWpiBlue() {
    var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    if (est != null && LimelightHelpers.validPoseEstimate(est)) {
      return est.pose;
    }
    return LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
  }

  public Pose2d getBotPoseWpiBlue() {
    var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");


    if (est != null && LimelightHelpers.validPoseEstimate(est)) {
        return est.pose;
    }

    return LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
}

  public Pose2d getRobotRelativeTargetPose() {
    double[] targetPoseArray =
        m_limeLightRightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[10]);
    return new Pose2d(
        targetPoseArray[2],
        targetPoseArray[0],
        Rotation2d.fromDegrees(targetPoseArray[4]));
  }

  public int getRightID(){
    return ((int) m_limeLightRightTable.getEntry("tid").getDouble(-1));
  }

  public int getLeftID(){
    return ((int) m_limeLightLeftTable.getEntry("tid").getDouble(-1));
  }

  public int getTopID(){
    return ((int) m_limeLightTopTable.getEntry("tid").getDouble(-1));
  }
 
  public int getTopIDCount(){
    return ((int) m_limeLightTopTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);
  }

  public int getRightIDCount(){
    return ((int) m_limeLightRightTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);
  }

  public int getLeftIDCount(){
    return ((int) m_limeLightLeftTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);
  }

  public boolean hasHubTarget() {
    int tid = getRightID();
    return frc.robot.vision.VisionConstants.HUB_TAG_IDS.contains(tid);
  }

  public double getDistanceToHubMeters(Optional<Alliance> alliance) {
    return drivetrain.getState().Pose.getTranslation().getDistance(VisionConstants.hubPosition(alliance));
}

  private void updateVisionOdometryIfHubTagVisible() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    int tagID = (int) LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME);
    if (tagID == -1) {
      return;
    }

    PoseEstimate est = null;
    if(alliance.isPresent() && alliance.get() == Alliance.Blue){
        est = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_NAME);
    } else if (alliance.isPresent() && alliance.get() == Alliance.Red){
        est = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_NAME);
        est.pose = new Pose2d(est.pose.getX(), est.pose.getY(), Rotation2d.fromDegrees(est.pose.getRotation().getDegrees() + 180));
    }

    if (est != null) {
      drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds);
    }
  }

  public double getRPSSmartDashboard() {
    return SmartDashboard.getNumber("Shooter RPS", 50.0);
  }

  @Override
  public void periodic() {
    updateVisionOdometryIfHubTagVisible();
    
   
    SmartDashboard.putNumber("Distance to hub meters", getDistanceToHubMeters(DriverStation.getAlliance()));
  }


  @Override
  public void simulationPeriodic() {
  }
}