package frc.robot.hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommand extends Command {

    private final HopperSubsystem m_hopperSubsystem;
    private final double m_targetMeters;
    private final double m_expoKV;

    // Uses default speed from HopperConstants.expoKV
    public HopperCommand(HopperSubsystem hopperSubsystem, double targetMeters) {
        this(hopperSubsystem, targetMeters, HopperConstants.KVelocity);
    }

    // expoKV controls peak speed: lower = faster (peak vel ≈ 12V / expoKV)
    public HopperCommand(HopperSubsystem hopperSubsystem, double targetMeters, double expoKV) {
        this.m_hopperSubsystem = hopperSubsystem;
        this.m_targetMeters = targetMeters;
        this.m_expoKV = expoKV;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        m_hopperSubsystem.goToPosition(m_targetMeters, m_expoKV);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Hopper Target Meters", m_targetMeters);
        SmartDashboard.putNumber("Hopper Current Pose Meters", m_hopperSubsystem.getPositionMeters());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.abs(m_hopperSubsystem.getPositionMeters() - m_targetMeters) < HopperConstants.positionToleranceMeters;
    }
}