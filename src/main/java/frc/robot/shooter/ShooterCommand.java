// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import frc.robot.shooter.LeftShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
  private final LeftShooterSubsystem m_leftShooterSubsystem;
  private final RightShooterSubsystem m_rightShooterSubsystem;

  private final double speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(RightShooterSubsystem m_rightShooterSubsytem, LeftShooterSubsystem m_leftShooterSubsystem, double speed) {
    // m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_leftShooterSubsystem = m_leftShooterSubsystem;
    this.m_rightShooterSubsystem = m_rightShooterSubsytem;
    this.speed = speed;
    addRequirements(m_leftShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_leftShooterSubsystem.shootFuel(speed);
    // m_leftShooterSubsystem.runVelocity(200);
    m_leftShooterSubsystem.runVelocityTorqueFOC(15);
    m_leftShooterSubsystem.printRPM();
    m_leftShooterSubsystem.printVoltageOutput();
    m_rightShooterSubsystem.runVelocityTorqueFOC(15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  // m_leftShooterSubsystem.shootFuel(0);
    //m_leftShooterSubsystem.runVelocity(0);
    m_leftShooterSubsystem.runVelocityTorqueFOC(0);
    m_rightShooterSubsystem.runVelocityTorqueFOC(0);

    // m_leftShooterSubsystem.resetVoltageOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
