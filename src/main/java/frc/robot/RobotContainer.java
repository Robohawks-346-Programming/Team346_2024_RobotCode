// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Pivot;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final XboxController driverControl = new XboxController(Constants.DriveConstants.DRIVER_CONTROLLER_PORT);
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Autos autos = new Autos();
  //public static final Pivot pivot = new Pivot();
  //public static final Shooter shooter = new Shooter();
  //public static final Intake intake = new Intake();
  public int isInverted = 1;
  
    public DoubleSupplier xAxis = () -> (driverControl.getLeftY());
    public DoubleSupplier yAxis = () -> (driverControl.getLeftX());
    public DoubleSupplier thetaAxis = () -> (driverControl.getRightX());

  public RobotContainer() {
    configureBindings();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Blue) {
          isInverted = 1;
        }
        else {
          isInverted = -1;
        }
    }
    drivetrain.setDefaultCommand(new Drive(drivetrain, xAxis, yAxis, thetaAxis, Constants.DriveConstants.MAX_MOVE_VELOCITY * isInverted, Constants.DriveConstants.MAX_TURN_VELOCITY * isInverted));
  }

  private void configureBindings() {
    new JoystickButton(driverControl, Button.kLeftBumper.value).onTrue(new SequentialCommandGroup(new InstantCommand(drivetrain::resetEncoders), new InstantCommand(drivetrain::zeroHeading)));
    new JoystickButton(driverControl, Button.kRightBumper.value).whileTrue(new Drive(drivetrain, xAxis, yAxis, thetaAxis, Constants.DriveConstants.MAX_MOVE_VELOCITY_FAST * isInverted, Constants.DriveConstants.MAX_TURN_VELOCITY_FAST * isInverted));
    new JoystickButton(driverControl, Button.kX.value).onTrue(new InstantCommand(drivetrain::resetFrontLeftAbsoluteEncoder));
    new JoystickButton(driverControl, Button.kY.value).onTrue(new InstantCommand(drivetrain::resetFrontRightAbsoluteEncoder));
    new JoystickButton(driverControl, Button.kA.value).onTrue(new InstantCommand(drivetrain::resetBackLeftAbsoluteEncoder));
    new JoystickButton(driverControl, Button.kB.value).onTrue(new InstantCommand(drivetrain::resetBackRightAbsoluteEncoder));
  }

  public Command getAutonomousCommand() {
    return autos.returnAutos();
  }
}
