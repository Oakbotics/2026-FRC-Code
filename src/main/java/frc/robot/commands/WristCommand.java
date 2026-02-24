// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
   private final WristSubsystem m_subsystem;
   private final Angle m_angle; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristCommand(WristSubsystem subsystem, Angle angle) {
    this.m_subsystem = subsystem;
    this.m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.goToSetpoint(m_angle);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     SmartDashboard.putNumber("currentWristAngle: ", m_subsystem.getPosition().magnitude());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}