// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.ElasticDashboard;
import frc.robot.drive.TunerConstants;
import frc.robot.elevator.ElevatorCommand;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.hopper.HopperCommand;
import frc.robot.hopper.HopperConstants;
import frc.robot.hopper.HopperExtendCommandGroup;
import frc.robot.hopper.HopperFeedShootCommand;
import frc.robot.hopper.HopperRetractCommandGroup;
import frc.robot.hopper.HopperSubsystem;
import frc.robot.hopper.HopperToggleCommand;
import frc.robot.intake.IntakeAutoStartCommandGroup;
// import frc.robot.intake.IntakeAutoStartCommandGroup;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.OutakeCommand;
import frc.robot.roller.RollerSubsystem;
import frc.robot.drive.AlignToTrench;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.AlignRotationToHubOdometry;
import frc.robot.vision.LimeLightSubsystem;
import frc.robot.vision.ResetOdometryLimelight;
import frc.robot.vision.ShootFromHubDistance;
import frc.robot.shooter.KickerCommandGroup;
import frc.robot.shooter.KickerRollerCommand;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;
import frc.robot.shooter.ShooterCommand;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    public double shooterRPS = 5.0;
    boolean isPressed;
    private final LeftShooterSubsystem m_leftShooterSubsystem = new LeftShooterSubsystem();
    private final RightShooterSubsystem m_rightShooterSubsystem = new RightShooterSubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem(drivetrain);
    private final ElasticDashboard elastic_dashboard = new frc.robot.util.ElasticDashboard(drivetrain, m_limeLightSubsystem);
    BooleanSupplier isShooting = () -> m_kickerSubsystem.getStatorCurrentAmps() > 15.0;
    double speedMultiplier;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                boolean isPressed = joystick.leftBumper().getAsBoolean();
                speedMultiplier = isPressed ? 0.45 : 0.8; 
                
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMultiplier) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMultiplier) 
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        joystick.rightTrigger().whileTrue(
            new ShooterCommand(m_rightShooterSubsystem, m_leftShooterSubsystem, () -> shooterRPS)
        ).onFalse(new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended));

        joystick.rightBumper().whileTrue(new KickerRollerCommand(m_kickerSubsystem, m_rollerSubsystem));

        joystick.povUp().onTrue(new InstantCommand(() -> shooterRPS = MathUtil.clamp(shooterRPS + 1, 1.0, 70.0)));
        joystick.povDown().onTrue(new InstantCommand(() -> shooterRPS = MathUtil.clamp(shooterRPS - 1, 1.0, 70.0)));

        joystick.povLeft().onTrue(new ResetOdometryLimelight(drivetrain));
        joystick.povRight().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
        joystick.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, 12));
        joystick.x().whileTrue(new OutakeCommand(m_intakeSubsystem, m_rollerSubsystem, 12));
        joystick.y().onTrue(new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended));
        joystick.a().onTrue(new HopperCommand(m_hopperSubsystem, 0.15));
        // joystick.b().onTrue(new HopperRetractCommandGroup(m_hopperSubsystem));
        // joystick.b().onTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.fullyExtended));

        // joystick.a().onTrue(new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended));
        // joystick.povRight().onTrue(new InstantCommand(() ->  m_hopperSubsystem.zeroHopper()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}