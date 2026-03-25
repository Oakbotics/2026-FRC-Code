// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import static edu.wpi.first.units.Units.Rotations;

import frc.robot.wrist.WristSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class WristCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
   private final WristSubsystem m_subsystem;
   private final Angle m_angle; 

  public WristCommand(WristSubsystem subsystem, Angle angle) {
    this.m_subsystem = subsystem;
    this.m_angle = angle;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.goToSetpoint(m_angle);
  }

  @Override
  public void execute() {
    m_subsystem.goToSetpoint(m_angle);
    SmartDashboard.putNumber("currentWristAngle: ", m_subsystem.getPosition().magnitude());
  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
    double currentRotations = m_subsystem.getPosition().in(Rotations);
    double targetRotations = m_angle.in(Rotations);
    return Math.abs(currentRotations - targetRotations) < WristConstants.positionToleranceRotations;
  }
}