package frc.robot.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.ShooterConstants;

public class KickerCommandGroup extends Command {
    private enum FeedState {
        feeding,
        unjamReverse,
        unjamRecovery
    }

    private final KickerSubsystem m_kickerSubsystem;
    private final HopperSubsystem m_hopperSubsystem;
    private Timer superCoderTimer = new Timer();
    private Debouncer superCoderJamDebouncer = new Debouncer(ShooterConstants.jamDebounceSec);
    private FeedState state = FeedState.feeding;
    private int jamCount = 0;

    public KickerCommandGroup(KickerSubsystem m_kickerSubsystem, HopperSubsystem m_hopperSubsystem){

        this.m_hopperSubsystem = m_hopperSubsystem;
        this.m_kickerSubsystem = m_kickerSubsystem;

        addRequirements(m_kickerSubsystem, m_hopperSubsystem);
    }

    private void restartTimer(){
        superCoderTimer.stop();
        superCoderTimer.reset();
        superCoderTimer.start();
    }

    private void sendStateToSmartDashboard(){
        SmartDashboard.putString("Current State", state.name());
        SmartDashboard.putNumber("Amount of Times Jammed: ", jamCount);
    }

    @Override
    public void initialize(){
        state = FeedState.feeding;
        jamCount = 0;
        superCoderTimer.stop();
        superCoderTimer.start();
        sendStateToSmartDashboard();
    }

    @Override
    public void execute(){
        boolean jamDetected = superCoderJamDebouncer.calculate(m_hopperSubsystem.getStatorCurrentAmps() > ShooterConstants.hopperJamCurrentAmps);

        //Super coder smart dahsboard stuff for super coder debugging
        SmartDashboard.putBoolean("DoWeHaveJam?????", jamDetected);
        SmartDashboard.putNumber("SuperKickerCurrentAMPS", m_kickerSubsystem.getStatorCurrentAmps());
        SmartDashboard.putNumber("SuperKickerVelocityRPS", m_kickerSubsystem.getVelovcityRps());
        SmartDashboard.putNumber("SuperHopperVelocityRPS", m_hopperSubsystem.getVelovcityRps());

        switch(state){
            case feeding:
                m_hopperSubsystem.feedTowardShooter(ShooterConstants.hopperFeedPercent);
                m_kickerSubsystem.feedTowardShooter(ShooterConstants.kickerFeedPercent);

                if (jamDetected){
                    jamCount++;
                    state = FeedState.unjamReverse;
                    restartTimer();
                    sendStateToSmartDashboard();
                }
                break;
            
            case unjamReverse:
                m_hopperSubsystem.reverseFromShooter(ShooterConstants.hopperReversePercent);

                if (superCoderTimer.hasElapsed(ShooterConstants.unjamReverseSec)){
                    state = FeedState.unjamRecovery;
                    restartTimer();
                    sendStateToSmartDashboard();
                }
                break;

            case unjamRecovery:
                m_hopperSubsystem.feedTowardShooter(ShooterConstants.hopperRecoveryPercent);
                m_kickerSubsystem.feedTowardShooter(ShooterConstants.kickerRecorveryPercent);

                if (superCoderTimer.hasElapsed(ShooterConstants.unjamRecoverySec)){
                    state = FeedState.feeding;
                    sendStateToSmartDashboard();
                }
                break;

        }

    }

    @Override
    public void end(boolean interrupted){
        m_hopperSubsystem.stop();
        m_kickerSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
