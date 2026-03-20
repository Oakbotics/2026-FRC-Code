// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.shooter.ShooterCommand;
import frc.robot.drive.TunerConstants;
import frc.robot.hopper.HopperCommand;
import frc.robot.hopper.HopperSubsystem;
import frc.robot.hopper.KickerCommandGroup;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.led.LEDSubsystem;
//import frc.robot.commands.IntakeCommandGroup;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.AlignRotationToHubOdometry;
import frc.robot.vision.LimeLightSubsystem;
import frc.robot.vision.ResetOdometryLimelight;
import frc.robot.vision.ShootFromHubDistance;
import frc.robot.wrist.WristAgitateCommandGroup;
import frc.robot.wrist.WristCommand;
import frc.robot.wrist.WristSubsystem;
import frc.robot.shooter.KickerCommand;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;
import frc.robot.shooter.ShootOnMoveAutoCommandGroup;
import frc.robot.shooter.ShootOnMoveToHub;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Angle angleDown = Degrees.of(140);
    private Angle angleUp = Degrees.of(10);
    private final LeftShooterSubsystem m_leftShooterSubsystem = new LeftShooterSubsystem();
    private final RightShooterSubsystem m_rightShooterSubsystem = new RightShooterSubsystem();
    // private final ShootFromHubDistance shootFromHubDistance = new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem);
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final WristSubsystem m_wristSubsystem = new WristSubsystem();
    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem(drivetrain);
    private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(() -> drivetrain.getState().Pose,joystick.getHID());   
    // private final SendableChooser<Command> m_autoChooser;


    public RobotContainer() {
        NamedCommands.registerCommand("ResetOdometryLimelight", new ResetOdometryLimelight(drivetrain));
        NamedCommands.registerCommand("AlignRotationToHubOdometry", new AlignRotationToHubOdometry( 
            drivetrain,
            m_limeLightSubsystem,
            () -> MathUtil.applyDeadband(joystick.getLeftY(), 0.10) * MaxSpeed,
            () -> MathUtil.applyDeadband(joystick.getLeftX(), 0.10) * MaxSpeed
        ));
        NamedCommands.registerCommand("ShootOnMoveAuto", new ShootOnMoveAutoCommandGroup(
            m_rightShooterSubsystem,
            m_leftShooterSubsystem,
            m_kickerSubsystem,
            drivetrain,
            m_limeLightSubsystem
        ));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSubsystem, 1));
        NamedCommands.registerCommand("WristCommand", new WristCommand(m_wristSubsystem, angleDown));
        NamedCommands.registerCommand("ShootFromHubDistance", new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem));
        
        Pose2d target = new Pose2d(drivetrain.getState().Pose.getX() + 1.0, drivetrain.getState().Pose.getY(), drivetrain.getState().Pose.getRotation());

        // m_autoChooser = AutoBuilder.buildAutoChooser();
        // m_autoChooser.setDefaultOption("MoveBack", AutoBuilder.buildAuto("MoveBack"));
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // joystick.povLeft().onTrue(new ResetOdometryLimelight(drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.a().whileTrue(new ShooterCommand(m_rightShooterSubsystem, m_leftShooterSubsystem, () -> m_limeLightSubsystem.getRPSSmartDashboard()));
        joystick.b().whileTrue(new KickerCommandGroup(m_kickerSubsystem, m_hopperSubsystem));
        joystick.x().whileTrue(new WristCommand(m_wristSubsystem, angleDown));
        joystick.y().onTrue(new WristCommand(m_wristSubsystem, angleUp));
        joystick.leftBumper().whileTrue(new WristAgitateCommandGroup(m_wristSubsystem));
        // joystick.b().onTrue(new WristCommand(m_wristSubsystem, angleUp));
        joystick.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, 5));
        joystick.povDown().onTrue(new ResetOdometryLimelight(drivetrain));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // joystick.rightBumper().whileTrue(new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem));

        // joystick.rightTrigger().whileTrue(
        //     new AlignRotationToHubOdometry(
        //         drivetrain,
        //         m_limeLightSubsystem,
        //         () -> MathUtil.applyDeadband(-joystick.getLeftY(), 0.10) * MaxSpeed,
        //         () -> MathUtil.applyDeadband(-joystick.getLeftX(), 0.10) * MaxSpeed
        //     )
        // );

        joystick.rightTrigger().whileTrue(
            new ShootOnMoveToHub(
                drivetrain, 
                m_leftShooterSubsystem, 
                m_rightShooterSubsystem, 
                () -> drivetrain.getState().Speeds.vxMetersPerSecond,
                () -> drivetrain.getState().Speeds.vyMetersPerSecond
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
