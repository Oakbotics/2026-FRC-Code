package frc.robot.wrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;

public class WristAgitateCommandGroup extends SequentialCommandGroup {
    Angle angleUp = Degrees.of(70);
    
    public WristAgitateCommandGroup(WristSubsystem m_wristSubsystem, IntakeSubsystem m_intakeSubsystem) {
        addCommands(
            
                new ParallelCommandGroup(
                    
                    new SequentialCommandGroup(
                        new WristCommand(m_wristSubsystem,  angleUp)
                
                    ),
                    new IntakeCommand(m_intakeSubsystem, 10)
                
            )
        );
    }
}