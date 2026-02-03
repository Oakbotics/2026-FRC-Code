package frc.robot.commands.autoCommand;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(CommandSwerveDrivetrain drivetrain) {
        Pose2d startingPose = new Pose2d(6.000 , 4.000 , Rotation2d.fromDegrees(0));
        Pose2d targetPose = new Pose2d(
            drivetrain.getPose().getX(),
            drivetrain.getPose().getY() + 1.0,
            drivetrain.getPose().getRotation()
        );

        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(startingPose), drivetrain),
            drivetrain.findPathToPose(targetPose
            ));

            
    }
}