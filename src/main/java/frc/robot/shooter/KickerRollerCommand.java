package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.roller.RollerSubsystem;

public class KickerRollerCommand extends Command {

    private final KickerSubsystem m_kickerSubsystem;
    private final RollerSubsystem m_rollerSubsystem;

    public KickerRollerCommand(KickerSubsystem m_kickerSubsystem, RollerSubsystem m_rollerSubsystem){

        this.m_rollerSubsystem = m_rollerSubsystem;
        this.m_kickerSubsystem = m_kickerSubsystem;

        addRequirements(m_kickerSubsystem, m_rollerSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {
        m_rollerSubsystem.setSpeed(ShooterConstants.hopperFeedPercent);
        m_kickerSubsystem.setKickerSpeed(ShooterConstants.kickerFeedPercent); 
    }
    
    @Override
    public void end(boolean interrupted){
        m_rollerSubsystem.stop();
        m_kickerSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}