// // // // // Copyright (c) FIRST and other WPILib contributors.
// // // // Open Source Software; you can modify and/or share it under the terms of
// // // // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Intake;

// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Intake;
// import frc.robot.RobotState;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// /** An example command that uses an example subsystem. */
// public class GroundIntake extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//     private Intake intake = RobotContainer.intake;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public GroundIntake() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     intake.runIntake();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intake.stopIntake();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }