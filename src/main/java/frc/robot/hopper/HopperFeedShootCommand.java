package frc.robot.hopper;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperFeedShootCommand extends Command {

    private final HopperSubsystem m_hopper;
    private final Timer m_feedTimer = new Timer();
    private boolean m_feeding;

    public HopperFeedShootCommand(HopperSubsystem hopper) {
        this.m_hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        m_hopper.setCruiseVelocity(3);
        m_hopper.goToPosition(HopperConstants.fullyRetracted);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_feedTimer.stop();
    }

    @Override
    public boolean isFinished() {
        if (!m_feeding) return false;
        return Math.abs(m_hopper.getPositionMeters() - HopperConstants.fullyRetracted) < HopperConstants.positionToleranceMeters;
    }
}