package frc.robot.commands;

import javax.sound.sampled.LineEvent;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.commands.MoveOdometry;
import frc.robot.commands.AlignToHubLimelight;

public class AlignAuto extends SequentialCommandGroup {

    private final LimeLightSubsystem limelight;

    public AlignAuto(CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limelight) {

        this.limelight = limelight;
        
        Pose2d startingPose = new Pose2d(3.389, 4.003, new Rotation2d(0));

        Pose2d targetPose = new Pose2d(
            startingPose.getX() - 1.116,
            startingPose.getY(),
            startingPose.getRotation()
        );

        Pose2d targetPose1 = new Pose2d(
            targetPose.getX(),
            targetPose.getY() + 0.5,
            targetPose.getRotation()
        );

        addCommands(
            drivetrain.resetPoseCommand(startingPose),

            new MoveOdometry(drivetrain, targetPose),
            new MoveOdometry(drivetrain, targetPose1),

            new AlignToHubLimelight(limelight, drivetrain)
        );
    }
}