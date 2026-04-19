package frc.robot.hopper;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class HopperRetractCommandGroup extends SequentialCommandGroup {
    public HopperRetractCommandGroup(HopperSubsystem m_hopperSubsystem) {
        addCommands(
           new SequentialCommandGroup(
                new HopperCommand(m_hopperSubsystem, HopperConstants.fullyRetracted)//.withTimeout(3.0),
                //new InstantCommand(() -> m_hopperSubsystem.applyIdleConfigs())
           )
        );
    }   
}