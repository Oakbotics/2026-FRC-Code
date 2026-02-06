package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.AlignToHubOdometry;

public class RotatePositionAuto extends SequentialCommandGroup {

    public RotatePositionAuto(CommandSwerveDrivetrain drivetrain) {
        
        Pose2d startingPose = new Pose2d(8.0, 1.0, new Rotation2d(0));

        Pose2d targetPose = new Pose2d(
            startingPose.getX() + 1.0,
            startingPose.getY(),
            startingPose.getRotation()
        );

        Pose2d targetPose1 = new Pose2d(
            targetPose.getX() + 2.0,
            targetPose.getY() + 2.0,
            targetPose.getRotation().plus(Rotation2d.fromDegrees(45))
        );

        Pose2d targetPose2 = new Pose2d(
            targetPose1.getX() - 2.0,
            targetPose1.getY() - 2.0,
            targetPose1.getRotation().plus(Rotation2d.fromDegrees(-45))
        );

        Pose2d targetPose3 = new Pose2d(
            targetPose2.getX() - 1.0,
            targetPose2.getY(),
            targetPose2.getRotation()
        );

        addCommands(
            drivetrain.resetPoseCommand(startingPose),

            new AlignToHubOdometry(drivetrain, targetPose),
            new AlignToHubOdometry(drivetrain, targetPose1),
            new AlignToHubOdometry(drivetrain, targetPose2),
            new AlignToHubOdometry(drivetrain, targetPose3)
        );
    }
}