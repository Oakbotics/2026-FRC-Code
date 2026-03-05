// package frc.robot.intake; 

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import frc.robot.wrist.WristCommand;
// import frc.robot.wrist.WristSubsystem;
// import frc.robot.intake.IntakeCommand;



// public class IntakeCommandGroup extends SequentialCommandGroup {
//   double speed = 1.0;
//   public IntakeCommandGroup(WristSubsystem wrist, IntakeSubsystem intake, RollerSubsystem roller, Angle angleDown) {
//     addCommands(
//         // Drive forward the specified distance
        
//         new SequentialCommandGroup(
//             new WristCommand(wrist, angleDown),
//             new ParallelCommandGroup(
//             new IntakeCommand(intake, speed),
//             new RollerCommand(roller, speed);
//           );
//         );
//       );

//   }

// }