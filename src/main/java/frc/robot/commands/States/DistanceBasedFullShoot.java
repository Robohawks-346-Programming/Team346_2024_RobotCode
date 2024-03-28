package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.RotateToHeading;
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
          RobotContainer.pivot.distanceBasedArmPivot(),
          //new InstantCommand(RobotContainer.pivot::stopPivot),
          new ParallelCommandGroup(
          new ShootSpeaker(),
          new RotateToHeading()
          
          )
      )
      
    );
  }



}
