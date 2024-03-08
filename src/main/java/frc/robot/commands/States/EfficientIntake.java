package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeArm;
import frc.robot.commands.Intake.IntakeFull;

public class EfficientIntake extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EfficientIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SequentialCommandGroup(
      new IntakeFull(),
      new IntakeArm()
      )
    );
  }
}
