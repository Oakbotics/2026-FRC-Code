package frc.robot.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.shooter.KickerCommand;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.LeftShooterSubsystem;
import frc.robot.shooter.RightShooterSubsystem;

public class KickerCommandGroup extends ParallelCommandGroup {
    public KickerCommandGroup(KickerSubsystem m_kickerSubsystem, HopperSubsystem m_hopperSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new HopperCommand(m_hopperSubsystem, 1.0),
                new KickerCommand(m_kickerSubsystem, 100)
            )
        );
    }   
}