package frc.robot.hopper;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperFeedShootCommand extends Command {

    private final HopperSubsystem m_hopper;
    private final BooleanSupplier m_isShootingSupplier;
    private final Timer m_feedTimer = new Timer();
    private boolean m_feeding;

    public HopperFeedShootCommand(HopperSubsystem hopper, BooleanSupplier isShootingSupplier) {
        this.m_hopper = hopper;
        this.m_isShootingSupplier = isShootingSupplier;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        m_feeding = false;
        m_feedTimer.stop();
        m_feedTimer.reset();
        m_hopper.goToPosition(m_hopper.getPositionMeters());
    }

    @Override
    public void execute() {
        if (!m_feeding) {
            m_hopper.goToPosition(m_hopper.getPositionMeters());

            if (m_isShootingSupplier.getAsBoolean()) {
                m_feeding = true;
                m_hopper.setCruiseVelocity(HopperConstants.elevatorFeedingRPS);
                m_feedTimer.start();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_feedTimer.stop();
    }

    @Override
    public boolean isFinished() {
        if (!m_feeding) return false;
        return Math.abs(m_hopper.getPositionMeters() - HopperConstants.fullyRetracted) < HopperConstants.positionToleranceMeters || m_feedTimer.hasElapsed(HopperConstants.elevatorFeedingTimeLimit);
    }
}