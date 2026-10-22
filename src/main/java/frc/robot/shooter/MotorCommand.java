// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MotorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
  private final MotorSubsystem m_motorSubsystem;
  private final double speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MotorCommand(MotorSubsystem m_motorSubsystem, double speed) {
    this.m_motorSubsystem = m_motorSubsystem;
    this.speed = speed;
    addRequirements(m_motorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_motorSubsystem.intakeFuel(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motorSubsystem.intakeFuel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}