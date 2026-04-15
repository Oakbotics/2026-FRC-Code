package frc.robot.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.hopper.HopperCommand;
// import frc.robot.hopper.HopperConstants;
// import frc.robot.hopper.HopperFeedShootCommand;
// import frc.robot.hopper.HopperSubsystem;
// import frc.robot.intake.IntakeAutoStartCommandGroup;
// import frc.robot.intake.IntakeCommand;
// import frc.robot.intake.IntakeSubsystem;
// import frc.robot.roller.RollerSubsystem;

// public class KickerCommandGroup extends SequentialCommandGroup {
//     public KickerCommandGroup(KickerSubsystem m_kickerSubsystem, RollerSubsystem m_rollerSubsystem, IntakeSubsystem m_intakesubsystem, HopperSubsystem m_hopperSubsystem) {
//         addCommands(
//             new ParallelCommandGroup(
//                 new KickerRollerCommand(m_kickerSubsystem, m_rollerSubsystem),
//                 new SequentialCommandGroup(
//                     new WaitCommand(2.0),
//                     new RepeatCommand(
//                         new SequentialCommandGroup(
//                             new HopperCommand(m_hopperSubsystem, 0.20),
//                             new WaitCommand(0.25),
//                             new HopperCommand(m_hopperSubsystem, 0.05),
//                             new WaitCommand(0.25)
//                         )
//                     )
//                 ),
//                 new IntakeCommand(m_intakesubsystem, IntakeConstants.intakeTorqueCurrent)
//             )

//         );
        
//     }
// }


// package frc.robot.shooter;

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
import frc.robot.intake.IntakeAutoStartCommandGroup;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.roller.RollerSubsystem;

public class KickerCommandGroup extends SequentialCommandGroup {
    public KickerCommandGroup(KickerSubsystem m_kickerSubsystem, RollerSubsystem m_rollerSubsystem, IntakeSubsystem m_intakesubsystem, HopperSubsystem m_hopperSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new KickerRollerCommand(m_kickerSubsystem, m_rollerSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            new HopperCommand(m_hopperSubsystem, 0.25),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.225),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.20),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.175),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.15),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.125),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.1),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.075),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.05),
                            new WaitCommand(0.15),
                            new HopperCommand(m_hopperSubsystem, 0.025),
                            new WaitCommand(0.15)
                        )
                    )
                ),
                new IntakeCommand(m_intakesubsystem, IntakeConstants.intakeTorqueCurrent)
            )

        );
        
    }
}


