package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.hopper.HopperCommand;
import frc.robot.hopper.HopperConstants;
import frc.robot.hopper.HopperSubsystem;

public class IntakeAutoStartCommandGroup extends SequentialCommandGroup {
    public IntakeAutoStartCommandGroup(
        IntakeSubsystem m_intakeSubsystem,
        HopperSubsystem m_hopperSubsystem

    ) {

        addCommands(
            new SequentialCommandGroup(

                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, IntakeConstants.intakeTorqueCurrent).withTimeout(0.5)
                ),
                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, IntakeConstants.intakeTorqueCurrent),
                    new HopperCommand(m_hopperSubsystem, HopperConstants.fullyExtended).withTimeout(0.25)
                ) 
            )

   
        );
    }   
}