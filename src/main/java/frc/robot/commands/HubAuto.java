package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.MoveOdometry;

public class HubAuto extends SequentialCommandGroup {

    public HubAuto(CommandSwerveDrivetrain drivetrain) {
        
        Pose2d startingPose = new Pose2d(3.389, 4.003, new Rotation2d(0));

        Pose2d targetPose = new Pose2d(
            startingPose.getX() - 1.116,
            startingPose.getY(),
            startingPose.getRotation()
        );

        Pose2d targetPose1 = new Pose2d(
            targetPose.getX(),
            targetPose.getY() + 3.377,
            targetPose.getRotation()
        );

        Pose2d targetPose2 = new Pose2d(
            targetPose1.getX() + 5.921,
            targetPose1.getY(),
            targetPose1.getRotation().plus(Rotation2d.fromDegrees(-90))
        );

        Pose2d targetPose3 = new Pose2d(
            targetPose2.getX(),
            targetPose2.getY() - 1.879,
            targetPose2.getRotation()
        );

         Pose2d targetPose4 = new Pose2d(
            targetPose3.getX(),
            targetPose3.getY() + 1.879,
            targetPose3.getRotation()
        );

        Pose2d targetPose5 = new Pose2d(
            targetPose4.getX() - 5.921,
            targetPose4.getY(),
            targetPose4.getRotation().plus(Rotation2d.fromDegrees(+90))
        );

        Pose2d targetPose6 = new Pose2d(
            targetPose5.getX(),
            targetPose5.getY() - 3.377,
            targetPose5.getRotation()
        );

        Pose2d targetPose7 = new Pose2d(
            targetPose6.getX() + 1.116,
            targetPose6.getY(),
            targetPose6.getRotation()
        );

        addCommands(
            drivetrain.resetPoseCommand(startingPose),

            new MoveOdometry(drivetrain, targetPose),
            new MoveOdometry(drivetrain, targetPose1),
            new MoveOdometry(drivetrain, targetPose2),
            new MoveOdometry(drivetrain, targetPose3),
            new MoveOdometry(drivetrain, targetPose4),
            new MoveOdometry(drivetrain, targetPose5),
            new MoveOdometry(drivetrain, targetPose6),
            new MoveOdometry(drivetrain, targetPose7)

        );
    }
}