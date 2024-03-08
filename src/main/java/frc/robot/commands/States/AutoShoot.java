package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shoot.EjectSpeaker;
import frc.robot.commands.Shoot.ShootSpeaker;

public class AutoShoot extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SequentialCommandGroup(
        new ParallelRaceGroup(
                new ShootSpeaker(),
                new WaitCommand(0.3)),
        new ParallelRaceGroup(
                new EjectSpeaker(),
                new WaitCommand(0.3))
      )
    );
  }



}
