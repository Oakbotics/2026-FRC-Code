package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class ShootFarCommandGroup extends ParallelCommandGroup {
    public ShootFarCommandGroup(RightShooterSubsystem m_rightShooterSubsystem , LeftShooterSubsystem m_leftShooterSubsystem , KickerSubsystem m_kickerSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> m_rightShooterSubsystem.runVelocityTorqueFOC(1), m_rightShooterSubsystem),
                new RunCommand(() -> m_leftShooterSubsystem.runVelocityTorqueFOC(1), m_leftShooterSubsystem)

            ).withTimeout(2.0),

            new ParallelCommandGroup(
                new RunCommand(() -> m_rightShooterSubsystem.runVelocityTorqueFOC(1), m_rightShooterSubsystem),
                new RunCommand(() -> m_leftShooterSubsystem.runVelocityTorqueFOC(1), m_leftShooterSubsystem),
                new RunCommand(() -> m_kickerSubsystem.setKickerSpeed(0), m_kickerSubsystem)
            )  
        );
    }   
}