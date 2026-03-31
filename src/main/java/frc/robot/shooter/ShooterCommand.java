// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LeftShooterSubsystem m_leftShooterSubsystem;
  private final RightShooterSubsystem m_rightShooterSubsystem;

  private final DoubleSupplier speed;
  public ShooterCommand(RightShooterSubsystem m_rightShooterSubsytem, LeftShooterSubsystem m_leftShooterSubsystem, DoubleSupplier speed) {
    this.m_leftShooterSubsystem = m_leftShooterSubsystem;
    this.m_rightShooterSubsystem = m_rightShooterSubsytem;
    this.speed = speed;
    addRequirements(m_leftShooterSubsystem, m_rightShooterSubsytem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double rps = speed.getAsDouble();

    m_leftShooterSubsystem.runVelocityTorqueFOC(rps);
    m_rightShooterSubsystem.runVelocityTorqueFOC(rps);
    m_leftShooterSubsystem.printLeftRPM();
    m_rightShooterSubsystem.printRightRPM();
  }

  @Override
  public void end(boolean interrupted) {
    m_leftShooterSubsystem.setVoltage(0);
    m_rightShooterSubsystem.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}