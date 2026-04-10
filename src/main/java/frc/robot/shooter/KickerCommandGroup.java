package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.hopper.HopperCommand;
import frc.robot.hopper.HopperConstants;
import frc.robot.hopper.HopperFeedShootCommand;
import frc.robot.hopper.HopperSubsystem;
import frc.robot.roller.RollerSubsystem;

public class KickerCommandGroup extends SequentialCommandGroup {
    public KickerCommandGroup(KickerSubsystem m_kickerSubsystem, RollerSubsystem m_rollerSubsystem, HopperSubsystem m_hopperSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new KickerRollerCommand(m_kickerSubsystem, m_rollerSubsystem),
                new RepeatCommand(
                    new SequentialCommandGroup(
                        new HopperCommand(m_hopperSubsystem, 0.20),
                        new WaitCommand(0.25),
                        new HopperCommand(m_hopperSubsystem, 0.10),
                        new WaitCommand(0.25)
                    )
                )

            

            )
        );
        
    }
}