package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootFarCommandGroup extends SequentialCommandGroup {
    public ShootFarCommandGroup(RightShooterSubsystem m_rightShooterSubsystem, LeftShooterSubsystem m_leftShooterSubsystem, KickerSubsystem m_kickerSubsystem) {
        addCommands(
            new ShooterCommand(m_rightShooterSubsystem, m_leftShooterSubsystem, 80.0).withTimeout(2.0),
            new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new ShooterCommand(m_rightShooterSubsystem, m_leftShooterSubsystem, 80.0),
                new KickerCommand(m_kickerSubsystem, 0.7)
            )
        );
    }
}