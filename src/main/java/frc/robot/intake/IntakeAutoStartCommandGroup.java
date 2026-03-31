package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.wrist.WristCommand;
import frc.robot.wrist.WristSubsystem;

public class IntakeAutoStartCommandGroup extends SequentialCommandGroup {
    Angle angleDown = Degrees.of(131);
    public IntakeAutoStartCommandGroup(
        IntakeSubsystem m_intakeSubsystem,
        WristSubsystem m_wristSubsystem

    ) {

        addCommands(
            new SequentialCommandGroup(

                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, 15).withTimeout(0.75)
                ),
                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, 15),
                    new WristCommand(m_wristSubsystem, angleDown)
                ) 
            )

   
        );
    }   
}