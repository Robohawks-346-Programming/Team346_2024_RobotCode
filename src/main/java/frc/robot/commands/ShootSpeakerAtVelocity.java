// // // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Shooter;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// /** An example command that uses an example subsystem. */
// public class ShootSpeakerAtVelocity extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//     private double shooterSpeed;
//     private Shooter shooter = RobotContainer.shooter;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public ShootSpeakerAtVelocity(double speed) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     shooterSpeed = speed;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     shooter.setSpeakerShooterVelocity(shooterSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     new ParallelDeadlineGroup(new InstantCommand(shooter::ejectSpeaker), new WaitCommand(2));
//     shooter.setSpeakerShooterSpeed(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return shooter.getSpeakerShooterVelocity(shooterSpeed);
//   }
// }