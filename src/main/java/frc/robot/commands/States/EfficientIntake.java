package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.IntakeArm;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.commands.Pivot.DistancePivot;
import frc.robot.commands.Shoot.EjectSpeaker;
import frc.robot.commands.Shoot.ShootSpeaker;

public class EfficientIntake extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EfficientIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    if(Math.abs(RobotContainer.pivot.getPosition() - Constants.PivotConstants.HOME_PIVOT_ANGLE) < Constants.PivotConstants.PIVOT_ANGLE_THRESHOLD) {
      addCommands(
          new SequentialCommandGroup(
        new IntakeFull(),
        new IntakeArm()
        )
      );
    }
  }
}
