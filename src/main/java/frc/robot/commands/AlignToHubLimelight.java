// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionAlignConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveOdometry;

/** An example command that uses an example subsystem. */
public class AlignToHubLimelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
    LimeLightSubsystem limeLight;
    CommandSwerveDrivetrain drivetrain;

    private MoveOdometry moveCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToHubLimelight(LimeLightSubsystem m_limeLightSubsystem, CommandSwerveDrivetrain drivetrain) {
    // m_subsystem = subsystem;
    this.limeLight = m_limeLightSubsystem;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getState().Pose;
    
    double tx = LimelightHelpers.getTX(VisionAlignConstants.LIMELIGHT_NAME);
    
    Rotation2d targetRotation = currentPose.getRotation().plus(Rotation2d.fromDegrees(tx));
    Pose2d targetPose = new Pose2d(currentPose.getX(), currentPose.getY(), targetRotation);

    moveCommand = new MoveOdometry(drivetrain, targetPose);
    moveCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moveCommand != null && moveCommand.isFinished();
  }
}