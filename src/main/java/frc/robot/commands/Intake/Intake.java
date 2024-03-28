// // // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Intake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setVelocity(0, 0);
    RobotContainer.intake.toggleIsIntaking();
    RobotContainer.indexer.timer.reset();
    RobotContainer.indexer.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(RobotContainer.pivot.getDegrees() - Constants.PivotConstants.HOME_PIVOT_ANGLE) < Constants.PivotConstants.PIVOT_ANGLE_THRESHOLD) {
      RobotContainer.intake.runIntake();
      RobotContainer.indexer.startIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.indexer.stopIndex();
    RobotContainer.intake.stopIntake();
    RobotContainer.intake.toggleIsIntaking();
    RobotContainer.intake.toggleHasGamePiece();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooter.getLaserBreak();
  }
}