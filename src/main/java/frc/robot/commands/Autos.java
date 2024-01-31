// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.commands.Shoot.ShootSpeaker;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.commands.Shoot.ShootSpeaker;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public final class Autos {
  Drivetrain drivetrain;
  private final SendableChooser<Command> autoChooser;


  public Autos() {
    drivetrain = RobotContainer.drivetrain;
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

    autoChooser = AutoBuilder.buildAutoChooser();
    
    }

    public void addCommands() {
      NamedCommands.registerCommand("Intake", new IntakeFull());
      NamedCommands.registerCommand("Shoot Close", new ShootSpeaker(Constants.ShooterConstants.SPEAKER_CLOSE_MOTOR_SPEED));
    }

    public void addAutos() {
      autoChooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command returnAutos() {
        return autoChooser.getSelected();
    }



       
}