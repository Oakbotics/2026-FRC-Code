// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.example.ExampleSubsystem;
/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
  private final ExampleSubsystem m_exampleSubsystem;
  private final double speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem m_exampleSubsystem, double speed) {
    this.m_exampleSubsystem = m_exampleSubsystem;
    this.speed = speed;
    addRequirements(m_exampleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_exampleSubsystem.motorForward(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_exampleSubsystem.motorForward(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}