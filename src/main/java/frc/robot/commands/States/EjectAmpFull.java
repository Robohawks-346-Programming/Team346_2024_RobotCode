package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.Index;
import frc.robot.commands.Intake.IntakeArm;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.commands.Shoot.EjectAmp;

public class EjectAmpFull extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public EjectAmpFull() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SequentialCommandGroup(
      new ParallelRaceGroup(new Index(), new WaitCommand(0.1)),
      new EjectAmp()
      )
    );
  }
  
}
