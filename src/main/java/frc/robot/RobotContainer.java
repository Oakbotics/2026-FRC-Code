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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.shooter.ShooterCommand;
import frc.robot.drive.TunerConstants;
import frc.robot.hopper.HopperCommand;
import frc.robot.hopper.HopperSubsystem;
import frc.robot.hopper.KickerCommandGroup;
import frc.robot.intake.IntakeAutoStartCommandGroup;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.IntakeWristCommandGroup;
// import frc.robot.led.LEDSubsystem;
// import frc.robot.util.ElasticDashboard;
//import frc.robot.commands.IntakeCommandGroup;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.DrivePIDTunerCommand;
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
// import frc.robot.shooter.ShootOnMoveAutoCommandGroup;
import frc.robot.shooter.ShootOnMoveToHub;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Angle angleDown = Degrees.of(138);
    private Angle angleUp = Degrees.of(10);
    boolean isPressed;
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
    // private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(() -> drivetrain.getState().Pose,joystick.getHID());  
    // private final ElasticDashboard elastic_dashboard = new frc.robot.util.ElasticDashboard(drivetrain, m_limeLightSubsystem);
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ResetOdometryLimelight", new ResetOdometryLimelight(drivetrain));
        NamedCommands.registerCommand("AlignRotationToHubOdometry", new AlignRotationToHubOdometry( 
            drivetrain,
            m_limeLightSubsystem,
            () -> MathUtil.applyDeadband(joystick.getLeftY(), 0.10) * MaxSpeed,
            () -> MathUtil.applyDeadband(joystick.getLeftX(), 0.10) * MaxSpeed
        ));
        // NamedCommands.registerCommand("ShootOnMoveAuto", new ShootOnMoveAutoCommandGroup(
        //     m_rightShooterSubsystem,
        //     m_leftShooterSubsystem,
        //     m_kickerSubsystem,
        //     drivetrain,
        //     m_limeLightSubsystem
        // ));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_intakeSubsystem, 10).withTimeout(4));
        NamedCommands.registerCommand("STARTIntakeCommand", new IntakeAutoStartCommandGroup(m_intakeSubsystem, m_wristSubsystem).withTimeout(4));
        NamedCommands.registerCommand("WristCommand", new WristCommand(m_wristSubsystem, angleDown));
        NamedCommands.registerCommand("ShootFromHubDistance", new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem));
        NamedCommands.registerCommand("RunKickerHopper", new KickerCommandGroup(m_kickerSubsystem, m_hopperSubsystem));

        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.setDefaultOption("BackAuto", AutoBuilder.buildAuto("BackAuto"));

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {

                boolean isPressed = joystick.leftStick().getAsBoolean();
                double speedMultiplier = isPressed ? 0.3 : 0.8; 
                
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedMultiplier); // Drive counterclockwise with negative X (left)
            })
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

        // joystick.a().whileTrue(new ShooterCommand(m_rightShooterSubsystem, m_leftShooterSubsystem, () -> m_limeLightSubsystem.getRPSSmartDashboard()));
        joystick.rightTrigger().whileTrue(
            new ParallelCommandGroup(

                new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem),
                new AlignRotationToHubOdometry(
                    drivetrain,
                    m_limeLightSubsystem,
                    () -> MathUtil.applyDeadband(-joystick.getLeftY(), 0.10) * MaxSpeed,
                    () -> MathUtil.applyDeadband(-joystick.getLeftX(), 0.10) * MaxSpeed
                )
            )
        );

        // joystick.leftStick().onTrue(new InstantCommand(isPressed = false));
        joystick.povUp().whileTrue(new ShootFromHubDistance(m_leftShooterSubsystem, m_rightShooterSubsystem, m_limeLightSubsystem));

        joystick.rightBumper().whileTrue(new KickerCommandGroup(m_kickerSubsystem, m_hopperSubsystem));
        joystick.povLeft().onTrue(new ResetOdometryLimelight(drivetrain));
        joystick.povDown().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
        joystick.leftBumper().whileTrue(new WristAgitateCommandGroup(m_wristSubsystem, m_intakeSubsystem));
        joystick.leftTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, 10));
        joystick.b().onTrue(new WristCommand(m_wristSubsystem, angleDown));

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

        // joystick.leftTrigger().whileTrue(
        //     new ShootOnMoveToHub(
        //         drivetrain, 
        //         m_leftShooterSubsystem, 
        //         m_rightShooterSubsystem, 
        //         () -> MathUtil.applyDeadband(-joystick.getLeftY(), 0.10) * MaxSpeed,
        //         () -> MathUtil.applyDeadband(-joystick.getLeftX(), 0.10) * MaxSpeed
        //     )
        // );

        // joystick.leftBumper().whileTrue(new WristAgitateCommandGroup(m_wristSubsystem, m_intakeSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    // public void updateDashboard() {
    //     elastic_dashboard.update();
    // }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
        // return Commands.none();
        // return new PathPlannerAuto("MoveBack");
    }
}
