// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToHubOdometry extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Pose2d m_targetPose;

  private final PIDController xController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController yController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController rotController = new PIDController(1.5, 0.0, 0.0);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final double m_maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double m_maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  public AlignToHubOdometry(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    m_drivetrain = drivetrain;
    m_targetPose = targetPose;

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(0.05);                 // meters
    yController.setTolerance(0.05);                 // meters
    rotController.setTolerance(Math.toRadians(3.0)); // radians

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d current = m_drivetrain.getState().Pose;

    double vx = MathUtil.clamp(
        xController.calculate(current.getX(), m_targetPose.getX()),
        -m_maxSpeedMps, m_maxSpeedMps
    );

    double vy = MathUtil.clamp(
        yController.calculate(current.getY(), m_targetPose.getY()),
        -m_maxSpeedMps, m_maxSpeedMps
    );

    double omega = MathUtil.clamp(
        rotController.calculate(current.getRotation().getRadians(), m_targetPose.getRotation().getRadians()),
        -m_maxAngularRateRadPerSec, m_maxAngularRateRadPerSec
    );

    m_drivetrain.setControl(
        drive.withVelocityX(vx)
             .withVelocityY(vy)
             .withRotationalRate(omega)
    );

  }

  @Override
  public void end(boolean interrupted) {
  
    m_drivetrain.setControl(
        drive.withVelocityX(0.0)
             .withVelocityY(0.0)
             .withRotationalRate(0.0)
    );

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}