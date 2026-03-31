package frc.robot.hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommand extends Command {

    private final HopperSubsystem m_hopperSubsystem;
    private final double m_targetMeters;

    public HopperCommand(HopperSubsystem hopperSubsystem, double targetMeters) {
        this.m_hopperSubsystem = hopperSubsystem;
        this.m_targetMeters = targetMeters;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        m_hopperSubsystem.goToPosition(m_targetMeters);
    }

    @Override
    public void execute() {
        m_hopperSubsystem.goToPosition(m_targetMeters);
        SmartDashboard.putNumber("Elevator Target Meters", m_targetMeters);
        SmartDashboard.putNumber("Elevator Current Pose Meters", m_hopperSubsystem.getPositionMeters());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.abs(m_hopperSubsystem.getPositionMeters() - m_targetMeters) < HopperConstants.positionToleranceMeters;
    }
}