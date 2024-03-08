// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Shoot.EjectSpeaker;
import frc.robot.commands.Shoot.ShootSpeaker;
import frc.robot.commands.States.AutoShoot;
import frc.robot.commands.States.EfficientIntake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public final class Autos {
  Drivetrain drivetrain;
  Pivot pivot;
  private final SendableChooser<Command> autoChooser;


  public Autos() {
    drivetrain = RobotContainer.drivetrain;
    pivot = RobotContainer.pivot;
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

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
      NamedCommands.registerCommand("Intake", new EfficientIntake());
     NamedCommands.registerCommand("Shoot", new AutoShoot());
     NamedCommands.registerCommand("Test Shoot", new SequentialCommandGroup(new AutoShoot()));
     //new ParallelRaceGroup(Commands.runOnce(() -> pivot.moveArm(-60)), new WaitCommand(0.5))
    //  NamedCommands.registerCommand("Pivot Close", new SequentialCommandGroup(new EfficientIntake(), new ParallelRaceGroup(Commands.runOnce(() -> pivot.moveArm(-35)), new WaitCommand(0.5))));
    //  NamedCommands.registerCommand("Pivot Far", new SequentialCommandGroup(new EfficientIntake(), new ParallelRaceGroup(Commands.runOnce(() -> pivot.moveArm(-25)), new WaitCommand(0.5))));
    NamedCommands.registerCommand("Pivot Close", new AutoShoot());
    NamedCommands.registerCommand("Pivot Far", new AutoShoot());
    NamedCommands.registerCommand("Rev", new ShootSpeaker());
    NamedCommands.registerCommand("Eject",  new ParallelRaceGroup(new EjectSpeaker(),new WaitCommand(0.3)));
    }

    

       
}