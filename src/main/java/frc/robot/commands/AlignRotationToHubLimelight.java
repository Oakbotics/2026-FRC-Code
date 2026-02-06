// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionAlignConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;

/**
 * Rotates robot to face the HUB using Limelight TX angle.
 * Pass-through translation is applied from driver; this command only overrides omega.
 * Smooths TX input and applies deadband to reduce jerking while moving.
 */
public class AlignRotationToHubLimelight extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimeLightSubsystem limelight;

    private final DoubleSupplier driverVx;
    private final DoubleSupplier driverVy;

    private final PIDController headingPID;

    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double lastSeenTime = -1.0;
    private double atSetpointStart = -1.0;

    private Double desiredHeadingRad = null;

    // Smooth TX to reduce jerking
    private double filteredTx = 0.0;
    private final double txAlpha = 0.2; // smoothing factor, 0.0-1.0

    // Deadband for small rotation errors (in degrees)
    private final double deadbandDeg = 0.5;

    public AlignRotationToHubLimelight(
            LimeLightSubsystem limelight,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier vx,
            DoubleSupplier vy
    ) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.driverVx = vx;
        this.driverVy = vy;

        headingPID = new PIDController(
                4.0,
                0,
                0.5
        );

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(VisionAlignConstants.HEADING_TOLERANCE_DEG));
        headingPID.setIntegratorRange(VisionAlignConstants.HEADING_I_MIN, VisionAlignConstants.HEADING_I_MAX);

        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        lastSeenTime = -1.0;
        atSetpointStart = -1.0;
        desiredHeadingRad = null;
        filteredTx = 0.0;
        headingPID.reset();
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();

        // Pass-through driver translation
        final double vx = driverVx.getAsDouble();
        final double vy = driverVy.getAsDouble();
        double omegaCmd = 0.0;

        // Limelight horizontal angle to target
        double rawTx = -LimelightHelpers.getTX(VisionAlignConstants.LIMELIGHT_NAME);
        boolean hasTarget = LimelightHelpers.getTV(VisionAlignConstants.LIMELIGHT_NAME);

        if (hasTarget) {
            lastSeenTime = now;

            // Smooth TX
            filteredTx = txAlpha * rawTx + (1 - txAlpha) * filteredTx;

            Pose2d robotPose = limelight.getBotPoseRightWpiBlue();
            double currentHeading = robotPose.getRotation().getRadians();

            // Convert filtered TX to radians
            desiredHeadingRad = currentHeading + Math.toRadians(filteredTx);

            // Apply PID for rotation
            omegaCmd = headingPID.calculate(currentHeading, desiredHeadingRad);

            // Deadband: ignore tiny rotation errors
            double errorDeg = Math.toDegrees(desiredHeadingRad - currentHeading);
            if (Math.abs(errorDeg) < deadbandDeg) {
                omegaCmd = 0.0;
            }

            omegaCmd = MathUtil.clamp(
                    omegaCmd,
                    -VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC,
                     VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC
            );

            atSetpointStart = headingPID.atSetpoint() ? (atSetpointStart < 0 ? now : atSetpointStart) : -1.0;
        } else {
            // Brief loss: continue toward last desired heading
            boolean withinGrace = lastSeenTime > 0 && (now - lastSeenTime) <= VisionAlignConstants.LOST_TARGET_GRACE_SEC;
            if (withinGrace && desiredHeadingRad != null) {
                Pose2d robotPose = limelight.getBotPoseRightWpiBlue();
                double currentHeading = robotPose.getRotation().getRadians();
                omegaCmd = headingPID.calculate(currentHeading, desiredHeadingRad);

                double errorDeg = Math.toDegrees(desiredHeadingRad - currentHeading);
                if (Math.abs(errorDeg) < deadbandDeg) {
                    omegaCmd = 0.0;
                }

                omegaCmd = MathUtil.clamp(
                        omegaCmd,
                        -VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC,
                         VisionAlignConstants.MAX_OMEGA_RAD_PER_SEC
                );
            } else {
                atSetpointStart = -1.0;
            }
        }

        // Drive with driver translation, rotation overridden
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omegaCmd)
        );

        // Debug
        SmartDashboard.putBoolean("HasTarget", hasTarget);
        SmartDashboard.putNumber("TxDeg", rawTx);
        SmartDashboard.putNumber("FilteredTxDeg", filteredTx);
        SmartDashboard.putBoolean("AtSetpoint", headingPID.atSetpoint());
        if (desiredHeadingRad != null)
            SmartDashboard.putNumber("DesiredDeg", Math.toDegrees(desiredHeadingRad));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (atSetpointStart < 0) return false;
        double now = Timer.getFPGATimestamp();
        boolean recentlySeen = lastSeenTime > 0 && (now - lastSeenTime) <= VisionAlignConstants.LOST_TARGET_GRACE_SEC;
        if (!recentlySeen) return false;
        return (now - atSetpointStart) >= VisionAlignConstants.HOLD_TIME_SEC;
    }
}