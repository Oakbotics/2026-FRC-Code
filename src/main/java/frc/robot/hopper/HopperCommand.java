package frc.robot.hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommand extends Command {

    private final HopperSubsystem m_hopperSubsystem;
    private final double m_targetMeters;
    private final double m_cruiseVelocityRPS;
        
    public HopperCommand(HopperSubsystem hopperSubsystem, double targetMeters) {
        this(hopperSubsystem, targetMeters, HopperConstants.cruiseVelocityRPS);
    }
    
    public HopperCommand(HopperSubsystem hopperSubsystem, double targetMeters, double cruiseVelocityRPS) {
        this.m_hopperSubsystem = hopperSubsystem;
        this.m_targetMeters = targetMeters;
        this.m_cruiseVelocityRPS = cruiseVelocityRPS;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        m_hopperSubsystem.goToPosition(m_targetMeters, m_cruiseVelocityRPS);    
    }

    @Override
    public void execute() {
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