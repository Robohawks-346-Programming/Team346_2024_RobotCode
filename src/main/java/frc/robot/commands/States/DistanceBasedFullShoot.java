package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DistancePivot;
import frc.robot.commands.Shoot.EjectSpeaker;
import frc.robot.commands.Shoot.ShootSpeaker;

public class DistanceBasedFullShoot extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DistanceBasedFullShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ShootSpeaker(),
        new DistancePivot()
      ),
      new ParallelDeadlineGroup(new EjectSpeaker(), new WaitCommand(1))
      )
      
    );
  }



}
