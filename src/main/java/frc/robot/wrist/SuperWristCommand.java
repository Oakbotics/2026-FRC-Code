// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.wrist;

// import frc.robot.wrist.SuperWristSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class SuperWristCommand extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   // private final ExampleSubsystem m_subsystem;
//   private final SuperWristSubsystem m_wristSubsystem;
//   private final double m_position;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public SuperWristCommand(SuperWristSubsystem wristSubsystem) {

//     addRequirements(wristSubsystem);
//     m_wristSubsystem = wristSubsystem;
//     m_position = 0.0;
//   }

//   public SuperWristCommand(SuperWristSubsystem wristSubsystem, double position) {

//     addRequirements(wristSubsystem);
//     m_wristSubsystem = wristSubsystem;
//     m_position = position;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_wristSubsystem.wristRotateToPosition(m_position);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_wristSubsystem.printWristPosition();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (m_position - 2 < m_wristSubsystem.getWristAngle() && m_position + 2 > m_wristSubsystem.getWristAngle());
//   }
// }