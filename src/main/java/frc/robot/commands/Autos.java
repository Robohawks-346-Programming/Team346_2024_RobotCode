// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.commands.Shoot.EjectSpeaker;
import frc.robot.commands.Shoot.ShootSpeaker;
import frc.robot.commands.States.AutoShoot;
import frc.robot.commands.States.DistanceBasedFullShoot;
import frc.robot.commands.States.ShootCloseAuto;
import frc.robot.commands.States.ShootFarAuto;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public final class Autos {
  Drivetrain drivetrain;
  Pivot m_pivot;
  private final SendableChooser<Command> autoChooser;


  public Autos(Pivot pivot) {
    drivetrain = RobotContainer.drivetrain;
    m_pivot = pivot;
    AutoBuilder.configureHolonomic(
      drivetrain::getPose, 
      drivetrain::resetPose, 
      drivetrain::getSpeeds, 
      drivetrain::drive, 
      Constants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG, 
        () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
        drivetrain);

        registerCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Shoot Only", new SequentialCommandGroup(new ParallelRaceGroup(new WaitCommand(1), new ShootSpeaker()), new ParallelRaceGroup(new WaitCommand(1), new EjectSpeaker()) ));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Load a Choreo trajectory as a PathPlannerPath
PathPlannerPath traj1 = PathPlannerPath.fromChoreoTrajectory("5 Piece");
PathPlannerPath traj2 = PathPlannerPath.fromChoreoTrajectory("5 Piece.1");
PathPlannerPath traj3 = PathPlannerPath.fromChoreoTrajectory("5 Piece.2");
PathPlannerPath traj4 = PathPlannerPath.fromChoreoTrajectory("5 Piece.3");
PathPlannerPath traj5 = PathPlannerPath.fromChoreoTrajectory("5 Piece.4");
      }

    public Command getAutos(){
      return autoChooser.getSelected();
    }

    public Command findAmp(){
      Pose2d targetPose;
      if (DriverStation.getAlliance().get() == Alliance.Blue){
       targetPose = new Pose2d(1.75, 7.5, Rotation2d.fromDegrees(270));
      } else {
         targetPose = new Pose2d(14.74, 7.5, Rotation2d.fromDegrees(270));
      }

      Command pathfindingCommand = AutoBuilder.pathfindToPose(
       targetPose,
        new PathConstraints(
        3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
      return pathfindingCommand;
    }

    public void registerCommands(){
      NamedCommands.registerCommand("Intake", new IntakeFull());
     NamedCommands.registerCommand("Shoot", new AutoShoot());
     NamedCommands.registerCommand("Test Shoot", new ParallelRaceGroup(new EjectSpeaker(), new WaitCommand(0.4)));
     //new ParallelRaceGroup(Commands.runOnce(() -> pivot.moveArm(-60)), new WaitCommand(0.5))
    NamedCommands.registerCommand("Pivot Close", new SequentialCommandGroup(new IntakeFull(), new ParallelCommandGroup(m_pivot.moveArm(-35), new ParallelRaceGroup(new ShootSpeaker(), new WaitCommand(0.5)))));
    NamedCommands.registerCommand("Pivot Far", new SequentialCommandGroup(new IntakeFull(), new ParallelCommandGroup(m_pivot.moveArm(-30), new ShootSpeaker())));
    NamedCommands.registerCommand("Rev", new ShootSpeaker());
    //NamedCommands.registerCommand("Eject", new SequentialCommandGroup(new ParallelRaceGroup(new EjectSpeaker(),new WaitCommand(0.5)), m_pivot.moveArm(-55)));
    NamedCommands.registerCommand("Eject", new ShootCloseAuto());
    NamedCommands.registerCommand("Eject Far", new ShootFarAuto());
    }
}