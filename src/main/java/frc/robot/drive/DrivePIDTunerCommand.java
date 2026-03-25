package frc.robot.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivePIDTunerCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private final PIDController xPID = new PIDController(0, 0, 0);
    private final PIDController yPID = new PIDController(0, 0, 0);
    private final PIDController headingPID = new PIDController(0, 0, 0);

    private Pose2d startPose;
    private Pose2d goalPose;

    private static final double MAX_SPEED_MPS = 2.0;
    private static final double MAX_OMEGA_RAD_S = 4.0;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DrivePIDTunerCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        SmartDashboard.putNumber("TargetX", SmartDashboard.getNumber("TargetX", 2.0));
        SmartDashboard.putNumber("TargetY", SmartDashboard.getNumber("TargetY", 0.0));
        SmartDashboard.putNumber("TargetDeg", SmartDashboard.getNumber("TargetDeg", 0.0));

        SmartDashboard.putNumber("Drive_kP", SmartDashboard.getNumber("Drive_kP", 3.8));
        SmartDashboard.putNumber("Drive_kI", SmartDashboard.getNumber("Drive_kI", 0.0));
        SmartDashboard.putNumber("Drive_kD", SmartDashboard.getNumber("Drive_kD", 0.0));

        SmartDashboard.putNumber("Heading_kP", SmartDashboard.getNumber("Heading_kP", 5.0));
        SmartDashboard.putNumber("Heading_kI", SmartDashboard.getNumber("Heading_kI", 0.0));
        SmartDashboard.putNumber("Heading_kD", SmartDashboard.getNumber("Heading_kD", 0.0));
    }

    @Override
    public void initialize() {
        xPID.setPID(
            SmartDashboard.getNumber("Drive_kP", 3.8),
            SmartDashboard.getNumber("Drive_kI", 0.0),
            SmartDashboard.getNumber("Drive_kD", 0.0));
        yPID.setPID(
            SmartDashboard.getNumber("Drive_kP", 3.8),
            SmartDashboard.getNumber("Drive_kI", 0.0),
            SmartDashboard.getNumber("Drive_kD", 0.0));
        headingPID.setPID(
            SmartDashboard.getNumber("Heading_kP", 5.0),
            SmartDashboard.getNumber("Heading_kI", 0.0),
            SmartDashboard.getNumber("Heading_kD", 0.0));

        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        xPID.reset();
        yPID.reset();
        headingPID.reset();

        startPose = drivetrain.getState().Pose;

        double targetX = SmartDashboard.getNumber("TargetX", 2.0);
        double targetY = SmartDashboard.getNumber("TargetY", 0.0);
        double targetDeg = SmartDashboard.getNumber("TargetDeg", 0.0);
        double headingRad = startPose.getRotation().getRadians();

        double goalX = startPose.getX() + targetX * Math.cos(headingRad) - targetY * Math.sin(headingRad);
        double goalY = startPose.getY() + targetX * Math.sin(headingRad) + targetY * Math.cos(headingRad);
        double goalRad = headingRad + Math.toRadians(targetDeg);

        goalPose = new Pose2d(goalX, goalY, new edu.wpi.first.math.geometry.Rotation2d(goalRad));

        SmartDashboard.putNumber("GoalX", goalPose.getX());
        SmartDashboard.putNumber("GoalY", goalPose.getY());
        SmartDashboard.putNumber("GoalDeg", goalPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        Pose2d current = drivetrain.getState().Pose;

        double errorX = goalPose.getX() - current.getX();
        double errorY = goalPose.getY() - current.getY();
        double errorHeadingDeg = goalPose.getRotation().minus(current.getRotation()).getDegrees();

        double vx = MathUtil.clamp(
            xPID.calculate(current.getX(), goalPose.getX()), -MAX_SPEED_MPS, MAX_SPEED_MPS);
        double vy = MathUtil.clamp(
            yPID.calculate(current.getY(), goalPose.getY()), -MAX_SPEED_MPS, MAX_SPEED_MPS);
        double omega = MathUtil.clamp(
            headingPID.calculate(current.getRotation().getRadians(), goalPose.getRotation().getRadians()),
            -MAX_OMEGA_RAD_S, MAX_OMEGA_RAD_S);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega));

        SmartDashboard.putNumber("CurrentX", current.getX());
        SmartDashboard.putNumber("CurrentY", current.getY());
        SmartDashboard.putNumber("CurrentDeg", current.getRotation().getDegrees());
        SmartDashboard.putNumber("ErrorX", errorX);
        SmartDashboard.putNumber("ErrorY", errorY);
        SmartDashboard.putNumber("ErrorHeadingDeg", errorHeadingDeg);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());  
    }

    @Override
    public boolean isFinished() {
        return false; // runs until you release the button
    }
}