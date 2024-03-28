package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Shoot.EjectSpeaker;
import frc.robot.commands.Shoot.ShootSpeaker;

public class IntakeFull extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeFull() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SequentialCommandGroup(
        new Intake(),
        new ParallelRaceGroup(new ReverseIndex(), new WaitCommand(0.07))
      )
    );
  }



}
