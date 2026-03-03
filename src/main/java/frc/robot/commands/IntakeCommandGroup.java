package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.WristCommand;

import frc.robot.commands.IntakeCommand;



public class IntakeCommandGroup extends SequentialCommandGroup {
  double speed = 1.0;
  public IntakeCommandGroup(WristSubsystem wrist, IntakeSubsytem intake, RollerSubsystem roller, Angle angleDown) {
    addCommands(
        // Drive forward the specified distance
        
        new SequentialCommandGroup(
          new WristCommand(wrist, angleDown),
          new ParallelCommandGroup(
            new IntakeCommand(intake, speed),
            new RollerCommand(roller, speed);
          );
        );
      );

  }

}