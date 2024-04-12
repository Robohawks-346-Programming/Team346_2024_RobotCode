package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.RotateToHeading;
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
          new RotateToHeading(),
          RobotContainer.pivot.distanceBasedArmPivot(),
          new ParallelRaceGroup(new ShootSpeaker(), new WaitCommand(1.5))
          ),
        new ParallelRaceGroup(new EjectSpeaker(), new WaitCommand(0.4), new ShootSpeaker()),
          new InstantCommand(RobotContainer.indexer::stopIndex),
          new ParallelCommandGroup(
            new InstantCommand(RobotContainer.shooter::stopShooter),
            RobotContainer.pivot.moveArm(-55)
          )
          

      )
      
    );
  }



}
