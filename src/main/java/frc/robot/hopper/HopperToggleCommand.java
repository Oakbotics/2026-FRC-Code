package frc.robot.hopper;

import edu.wpi.first.wpilibj2.command.Command;

public class HopperToggleCommand extends Command {


    private static final double nearRetracted = HopperConstants.fullyRetracted + HopperConstants.positionToleranceMeters;
    private final HopperSubsystem m_hopper;
    private final double m_cruiseVelocityRPS;
    private double m_targetMeters;

    public HopperToggleCommand(HopperSubsystem hopper) {
        this(hopper, HopperConstants.elevatorFeedingRPS);
    }

    public HopperToggleCommand(HopperSubsystem hopper, double cruiseVelocityRPS) {
        this.m_hopper = hopper;
        this.m_cruiseVelocityRPS = cruiseVelocityRPS;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        double current = m_hopper.getPositionMeters();
        if (current <= nearRetracted) {
            m_targetMeters = HopperConstants.fullyExtended;
        } else {
            m_targetMeters = HopperConstants.fullyRetracted;
        }

        m_hopper.goToPosition(m_targetMeters, m_cruiseVelocityRPS);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_hopper.getPositionMeters() - m_targetMeters) < HopperConstants.positionToleranceMeters;
    }
}