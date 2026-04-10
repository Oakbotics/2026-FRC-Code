package frc.robot.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperFeedShootCommand extends Command {

    private final HopperSubsystem m_hopper;

    public HopperFeedShootCommand(HopperSubsystem hopper) {
        this.m_hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        m_hopper.goToPosition(HopperConstants.fullyRetracted, HopperConstants.elevatorFeedingRPS);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_hopper.getPositionMeters() - HopperConstants.fullyRetracted) < HopperConstants.positionToleranceMeters;
    }
}