// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.util.ElasticDashboard;
import frc.robot.drive.TunerConstants;
import frc.robot.hopper.HopperSubsystem;
import frc.robot.hopper.KickerCommandGroup;
import frc.robot.intake.IntakeAutoStartCommandGroup;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.OutakeCommand;
import frc.robot.drive.AlignToTrench;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.AlignRotationToHubOdometry;
import frc.robot.vision.LimeLightSubsystem;
import frc.robot.vision.ResetOdometryLimelight;
import frc.robot.vision.ShootFromHubDistance;
import frc.robot.wrist.WristAgitateCommandGroup;
import frc.robot.wrist.WristCommand;
import frc.robot.wrist.WristSubsystem;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private Angle angleDown = Degrees.of(121);
    private Angle angleUp = Degrees.of(74);
    boolean isPressed;
    private final LeftShooterSubsystem m_leftShooterSubsystem = new LeftShooterSubsystem();
    private final RightShooterSubsystem m_rightShooterSubsystem = new RightShooterSubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final WristSubsystem m_wristSubsystem = new WristSubsystem();
    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem(drivetrain);
    private final ElasticDashboard elastic_dashboard = new frc.robot.util.ElasticDashboard(drivetrain, m_limeLightSubsystem);
    private final SendableChooser<Command> m_autoChooser;
    double speedMultiplier;

    public RobotContainer() {
        NamedCommands.registerCommand("ResetOdometryLimelight", new ResetOdometryLimelight(drivetrain));
        NamedCommands.registerCommand("AlignRotationToHubOdometry", new AlignRotationToHubOdometry( 
            drivetrain,
            m_limeLightSubsystem,
            () -> MathUtil.applyDeadband(joystick.getLeftY(), 0.10) * MaxSpeed,
            () -> MathUtil.applyDeadband(joystick.getLeftX(), 0.10) * MaxSpeed
        ).withTimeout(1.0));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSubsystem, 15).withTimeout(4));
        NamedCommands.registerCommand("SlowIntakeCommand", new IntakeCommand(m_intakeSubsystem, 6).withTimeout(2));
        NamedCommands.registerCommand("Outake", new OutakeCommand(m_intakeSubsystem, 15).withTimeout(0.5));

        NamedCommands.registerCommand("STARTIntakeCommand", new IntakeAutoStartCommandGroup(m_intakeSubsystem, m_wristSubsystem).withTimeout(4));
        NamedCommands.registerCommand("WristCommand", new WristCommand(m_wristSubsystem, angleDown));
        NamedCommands.registerCommand("DumpWristCommand", new WristCommand(m_wristSubsystem, angleUp));
        NamedCommands.registerCommand("ShootFromHubDistance", new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem));
        NamedCommands.registerCommand("RunKickerHopper", new KickerCommandGroup(m_kickerSubsystem, m_hopperSubsystem));

        m_autoChooser = AutoBuilder.buildAutoChooser();
        
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                boolean isPressed = joystick.leftBumper().getAsBoolean();
                speedMultiplier = isPressed ? 0.45 : 0.8; 
                
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMultiplier) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMultiplier) 
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedMultiplier);
            })
        );

       
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.rightTrigger().whileTrue(
            new ParallelCommandGroup(

                new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new WristCommand(m_wristSubsystem, angleUp)
                ),
                new AlignRotationToHubOdometry(
                    drivetrain,
                    m_limeLightSubsystem,
                    () -> MathUtil.applyDeadband(-joystick.getLeftY(), 0.10) * MaxSpeed,
                    () -> MathUtil.applyDeadband(-joystick.getLeftX(), 0.10) * MaxSpeed
                ),
                new IntakeCommand(m_intakeSubsystem, 6)
            )
        ).onFalse(new WristCommand(m_wristSubsystem, angleDown));
        joystick.povUp().whileTrue(
            new ParallelCommandGroup(

                new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new WristCommand(m_wristSubsystem, angleUp)
                ),
                new IntakeCommand(m_intakeSubsystem, 6)
            )
        );

        joystick.rightBumper().whileTrue(new KickerCommandGroup(m_kickerSubsystem, m_hopperSubsystem));
        joystick.povLeft().onTrue(new ResetOdometryLimelight(drivetrain));
        joystick.povDown().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
        joystick.y().whileTrue(new WristAgitateCommandGroup(m_wristSubsystem, m_intakeSubsystem));
        joystick.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, 15));
        joystick.b().onTrue(new WristCommand(m_wristSubsystem, angleDown));
        joystick.x().whileTrue(new OutakeCommand(m_intakeSubsystem, 15));
        joystick.a().whileTrue(new AlignToTrench(drivetrain, () -> MathUtil.applyDeadband(-joystick.getLeftX(), 0.10) * MaxSpeed * speedMultiplier));

        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("RightTrenchCenter2CycleAuto");
        return new PathPlannerAuto("LeftTrenchCenter2CycleAuto");
        // return new PathPlannerAuto("BackAuto");
    }
}