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

public class WristAgitateCommandGroup extends SequentialCommandGroup {
    Angle angleUp = Degrees.of(100);
    Angle angleDown = Degrees.of(140);
    
    public WristAgitateCommandGroup(WristSubsystem m_wristSubsystem) {
        addCommands(
            new RepeatCommand(
                new SequentialCommandGroup(
                    new WristCommand(m_wristSubsystem, angleUp),
                    new WaitCommand(0.3),
                    new WristCommand(m_wristSubsystem, angleDown),
                    new WaitCommand(0.3)
                )
            )
        );
    }
}