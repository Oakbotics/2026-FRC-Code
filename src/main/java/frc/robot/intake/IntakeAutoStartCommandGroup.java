package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.vision.LimeLightSubsystem;
import frc.robot.wrist.WristCommand;
import frc.robot.wrist.WristSubsystem;

public class IntakeAutoStartCommandGroup extends SequentialCommandGroup {
    Angle angleDown = Degrees.of(138);
    public IntakeAutoStartCommandGroup(
        IntakeSubsystem m_intakeSubsystem,
        WristSubsystem m_wristSubsystem

    ) {

        addCommands(
            new SequentialCommandGroup(

                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, 10).withTimeout(1.0)
                ),
                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, 10),
                    new WristCommand(m_wristSubsystem, angleDown)
                ) 
            )

   
        );
    }   
}