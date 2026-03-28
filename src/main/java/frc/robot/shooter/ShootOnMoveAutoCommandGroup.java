// package frc.robot.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.drive.CommandSwerveDrivetrain;
// import frc.robot.vision.LimeLightSubsystem;

// public class ShootOnMoveAutoCommandGroup extends SequentialCommandGroup {
//     public ShootOnMoveAutoCommandGroup(
//         RightShooterSubsystem m_rightShooterSubsystem, 
//         LeftShooterSubsystem m_leftShooterSubsystem, 
//         KickerSubsystem m_kickerSubsystem, 
//         CommandSwerveDrivetrain drivetrain,
//         LimeLightSubsystem m_limeLightSubsystem) {

//         addCommands(
//             new ParallelDeadlineGroup(
//                 new WaitCommand(1),

//                 new ShootOnMoveToHub(
//                     drivetrain,
//                     m_leftShooterSubsystem, 
//                     m_rightShooterSubsystem, 
//                     () -> drivetrain.getState().Speeds.vxMetersPerSecond,
//                     () -> drivetrain.getState().Speeds.vyMetersPerSecond
//                 )
//             ),

//             new ParallelDeadlineGroup(
//                 new WaitCommand(3),
            
//                 new ShootOnMoveToHub(
//                     drivetrain,
//                     m_leftShooterSubsystem, 
//                     m_rightShooterSubsystem, 
//                     () -> drivetrain.getState().Speeds.vxMetersPerSecond,
//                     () -> drivetrain.getState().Speeds.vyMetersPerSecond
//                 ),

//                 new KickerCommand(m_kickerSubsystem, 100)
//             )  
//         );
//     }   
// }