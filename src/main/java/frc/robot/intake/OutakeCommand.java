// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.roller.RollerSubsystem;
import frc.robot.shooter.ShooterConstants;

/** An example command that uses an example subsystem. */
public class OutakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final RollerSubsystem m_rollerSubsystem;
  private final double speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OutakeCommand(IntakeSubsystem m_intakeSubsystem, RollerSubsystem m_rollerSubsystem, double speed) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_rollerSubsystem = m_rollerSubsystem;
    this.speed = speed;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.outakeFuel(speed);
    m_rollerSubsystem.setSpeed(-ShooterConstants.hopperFeedPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.outakeFuel(0);
    m_rollerSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}