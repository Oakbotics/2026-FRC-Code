package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class FollowTrajectory extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    private final double m_maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double m_maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final HolonomicDriveController controller =
        new HolonomicDriveController(
            new PIDController(3.8, 0, 0),
            new PIDController(3.8, 0, 0),
            new ProfiledPIDController(
                5, 0, 0,
                new TrapezoidProfile.Constraints(Math.PI, Math.PI)
            )
        );

    public FollowTrajectory(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        controller.getThetaController().enableContinuousInput(-Math.PI, Math.PI);

        TrajectoryConfig config = new TrajectoryConfig(3.0, 2.0);

        Pose2d start = new Pose2d(1.0, 5.0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(45));
        List<Translation2d> interiorWaypoints = List.of(new Translation2d(3.0, 6));

        trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetOdometry(trajectory.getInitialPose());
        timer.restart();    
    }

    @Override
    public void execute() {

        var desiredState = trajectory.sample(timer.get());

        var speeds = controller.calculate(
            drivetrain.getPose(),
            desiredState,
            desiredState.poseMeters.getRotation()
            //Rotation2d.fromDegrees(0)
        );

        double vx = MathUtil.clamp(speeds.vxMetersPerSecond, -m_maxSpeedMps, m_maxSpeedMps);
        double vy = MathUtil.clamp(speeds.vyMetersPerSecond, -m_maxSpeedMps, m_maxSpeedMps);
        double omega = MathUtil.clamp(speeds.omegaRadiansPerSecond, -m_maxAngularRateRadPerSec, m_maxAngularRateRadPerSec);

        SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omega);

        drivetrain.setControl(driveRequest);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveRequest.FieldCentric stopRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

        drivetrain.setControl(stopRequest);
    }
}