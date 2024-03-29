// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PivotToAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private double level;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PivotToAngle(double height) {
    // Use addRequirements() here to declare subsystem dependencies.
    level = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.pivot.moveArmToPosition(level);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.pivot.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.pivot.isAtPosition(level);
    //return RobotContainer.pivot.getArmPosition() >= 12;
  }
}