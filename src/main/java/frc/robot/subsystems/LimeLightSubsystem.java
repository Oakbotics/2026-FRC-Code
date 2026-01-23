// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimeLightSubsystem extends SubsystemBase {

  public NetworkTable m_limelightBottomTable; 
   public Pose2d m_closestTagPose;
  //Limelight Bottom: http://10.37.39.12
  
  /** Creates a new LimeLightSubsystem. */
  public LimeLightSubsystem() {
    m_limelightBottomTable = NetworkTableInstance.getDefault().getTable("limelight-right");
    m_limelightBottomTable.getEntry("pipeline").setNumber(0);
    // SmartDashboard.putData("Field", m_field);

  }
  public Pose2d getBotPoseBottomLL(){
    if(DriverStation.getAlliance().get() == Alliance.Red)
      m_limelightBottomTable.getEntry("pipeline").setNumber(0);
    else
      m_limelightBottomTable.getEntry("pipeline").setNumber(1);

    double[] botRotArray = m_limelightBottomTable.getEntry("botpose").getDoubleArray(new double[10]); 
    double[] botPoseArray = m_limelightBottomTable.getEntry("botpose_orb").getDoubleArray(new double[10]); 
    Pose2d botPose;
      if(DriverStation.getAlliance().get() == Alliance.Red)  botPose = new Pose2d(botPoseArray[0]+8.7736, botPoseArray[1]+4.0257, Rotation2d.fromDegrees(botRotArray[5] + 180));
      else botPose = new Pose2d(botPoseArray[0] + 8.7736, botPoseArray[1] + 4.0257, Rotation2d.fromDegrees(botRotArray[5]));
      return botPose;

  }

  public Pose2d getRobotRelativeTargetPose(){
    double[] targetPoseArray = m_limelightBottomTable.getEntry("targetpose_robotspace").getDoubleArray(new double[10]);
    Pose2d targetPose = new Pose2d(targetPoseArray[2], targetPoseArray[0], Rotation2d.fromDegrees(targetPoseArray[4])); 

    return targetPose;
  }

  public int getBottomID(){
    return ((int) m_limelightBottomTable.getEntry("tid").getDouble(-1));
  }

  public int getBottomIDCount(){
    return ((int) m_limelightBottomTable.getEntry("botpose_orb").getDoubleArray(new double[10])[7]);

  }

  public double getBottomLimelightTime(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right").timestampSeconds;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Bottom Pipline", m_limelightBottomTable.getEntry("pipeline").getNumber(-1).doubleValue());
  }

  @Override
  public void simulationPeriodic() {
  }
}