package frc.robot.hopper;

import edu.wpi.first.wpilibj2.command.Command;

public class HopperToggleCommand extends Command {


    private static final double nearRetracted = HopperConstants.fullyRetracted + HopperConstants.positionToleranceMeters;
    private final HopperSubsystem m_hopper;
    private double m_targetMeters;

    public HopperToggleCommand(HopperSubsystem hopper) {
        this.m_hopper = hopper;
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
    }

    @Override
    public void execute() {
        m_hopper.goToPosition(m_targetMeters);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_hopper.getPositionMeters() - m_targetMeters) < HopperConstants.positionToleranceMeters;
    }
}