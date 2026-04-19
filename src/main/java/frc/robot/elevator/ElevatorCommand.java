package frc.robot.elevator;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_targetMeters;
    private final double m_expoKV;

    // Uses default speed from elevatorConstants.expoKV
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetMeters) {
        this(elevatorSubsystem, targetMeters, ElevatorConstants.expoKV);
    }

    // expoKV controls peak speed: lower = faster (peak vel ≈ 12V / expoKV)
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetMeters, double expoKV) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_targetMeters = targetMeters;
        this.m_expoKV = expoKV;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.goToPosition(m_targetMeters, m_expoKV);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("elevator Target Meters", m_targetMeters);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.abs(m_elevatorSubsystem.getPositionMeters() - m_targetMeters) < ElevatorConstants.positionToleranceMeters;
    }
}