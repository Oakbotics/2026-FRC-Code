package frc.robot.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.KickerSubsystem;
import frc.robot.shooter.ShooterConstants;

public class KickerCommandGroup extends Command {

    private final KickerSubsystem m_kickerSubsystem;
    private final HopperSubsystem m_hopperSubsystem;
    private Timer superCoderTimer = new Timer();

    public KickerCommandGroup(KickerSubsystem m_kickerSubsystem, HopperSubsystem m_hopperSubsystem){

        this.m_hopperSubsystem = m_hopperSubsystem;
        this.m_kickerSubsystem = m_kickerSubsystem;

        addRequirements(m_kickerSubsystem, m_hopperSubsystem);
    }



    @Override
    public void initialize(){
        superCoderTimer.stop();
        superCoderTimer.start();
    
    }

    @Override
    public void execute(){
                m_hopperSubsystem.feedTowardShooter(ShooterConstants.hopperFeedPercent);
                m_kickerSubsystem.feedTowardShooter(ShooterConstants.kickerFeedPercent);

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
