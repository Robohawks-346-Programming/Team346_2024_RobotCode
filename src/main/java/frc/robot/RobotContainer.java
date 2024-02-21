// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Shoot.TestShooter;
import frc.robot.commands.Autos;
import frc.robot.commands.PivotToAngle;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Intake.IntakeArm;
import frc.robot.commands.Shoot.EjectAmp;
// import frc.robot.commands.PivotToAngle;
import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.swing.plaf.basic.BasicOptionPaneUI.ButtonAreaLayout;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController driverControl = new CommandXboxController(Constants.DriveConstants.DRIVER_CONTROLLER_PORT);
  private Trigger rightTrigger = driverControl.rightTrigger();
  private Trigger rightBumper = driverControl.rightBumper();
  private Trigger x = driverControl.x();
  private Trigger y = driverControl.y();
  private Trigger b = driverControl.b();
  private Trigger a = driverControl.a();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Autos autos = new Autos();
  public static final LEDs leds = new LEDs();
  public static final Pivot pivot = new Pivot();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  // public static final Intake intake = new Intake();
  // public static final Climber climber = new Climber();
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
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, xAxis, yAxis, thetaAxis, 
    Constants.DriveConstants.MAX_MOVE_VELOCITY * isInverted, 
    Constants.DriveConstants.MAX_TURN_VELOCITY * isInverted));

    autos.addAutos();
  }

  private void configureBindings() {
    rightBumper.onTrue(new InstantCommand(() -> {
      drivetrain.zeroHeading(); 
      drivetrain.setFieldToVehicle(
        new Pose2d(drivetrain.poseEstimator.getEstimatedPosition().getTranslation(),
        new Rotation2d(0)));
    }));
    rightTrigger.whileTrue(
      new TeleopDrive(drivetrain, xAxis, yAxis, thetaAxis, 
      Constants.DriveConstants.MAX_MOVE_VELOCITY_FAST * isInverted, 
      Constants.DriveConstants.MAX_TURN_VELOCITY_FAST * isInverted));
    x.whileTrue(new TestShooter());
    y.whileTrue(new IntakeArm());
    b.onTrue(new PivotToAngle(10));
    a.onTrue
    (new PivotToAngle(-55));

  }

  public Command getAutonomousCommand() {
    return autos.returnAutos();
  }
}
