// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.States.EfficientIntake;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Intake.IntakeArm;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.commands.Intake.Outake;
import frc.robot.commands.Pivot.PivotToAngle;
import frc.robot.commands.Shoot.EjectAmp;
import frc.robot.commands.Shoot.ShootSpeaker;
import frc.robot.commands.Pivot.PivotToAngle;
//import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
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
import edu.wpi.first.wpilibj.Joystick;
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
  public static final Joystick operatorControl = new Joystick(Constants.DriveConstants.OPERATOR_CONTROLLER_PORT);

  private Trigger rightTrigger = driverControl.rightTrigger();
  private Trigger rightBumper = driverControl.rightBumper();
  private Trigger x = driverControl.x();
  private Trigger y = driverControl.y();
  private Trigger b = driverControl.b();
  private Trigger a = driverControl.a();
  private Trigger leftTrigger = driverControl.leftTrigger();
  private Trigger leftBumper = driverControl.leftBumper();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Autos autos = new Autos();
  //public static final LEDs leds = new LEDs();
  public static final Pivot pivot = new Pivot();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  public static final Intake intake = new Intake();
  public static final Climber climber = new Climber();
  public int isInverted = 1;
  
    public DoubleSupplier xAxis = () -> (driverControl.getLeftY());
    public DoubleSupplier yAxis = () -> (driverControl.getLeftX());
    public DoubleSupplier thetaAxis = () -> (driverControl.getRightX());

    public static final JoystickButton BUTTON_1 = new JoystickButton(operatorControl, 1),
      BUTTON_2 = new JoystickButton(operatorControl, 2),
      BUTTON_3 = new JoystickButton(operatorControl, 3),
      BUTTON_4 = new JoystickButton(operatorControl, 4),
      BUTTON_5 = new JoystickButton(operatorControl, 5),
      BUTTON_6 = new JoystickButton(operatorControl, 6),
      BUTTON_7 = new JoystickButton(operatorControl, 7),
      BUTTON_8 = new JoystickButton(operatorControl, 8),
      BUTTON_9 = new JoystickButton(operatorControl, 9),
      BUTTON_10 = new JoystickButton(operatorControl, 10),
      BUTTON_11 = new JoystickButton(operatorControl, 11),
      BUTTON_12 = new JoystickButton(operatorControl, 12),
      BUTTON_13 = new JoystickButton(operatorControl, 13),
      BUTTON_14 = new JoystickButton(operatorControl, 14),
      BUTTON_15 = new JoystickButton(operatorControl, 15),
      BUTTON_16 = new JoystickButton(operatorControl, 16);

  public RobotContainer() {
    configureBindings();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get().equals(Alliance.Blue)) {
          isInverted = 1;
        }
        else {
          isInverted = -1;
        }
    }
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, xAxis, yAxis, thetaAxis, 
    Constants.DriveConstants.MAX_MOVE_VELOCITY * isInverted, 
    Constants.DriveConstants.MAX_TURN_VELOCITY * isInverted));
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
    BUTTON_1.whileTrue(new EfficientIntake());
    BUTTON_2.whileTrue(new ShootSpeaker());
    BUTTON_3.whileTrue(new Outake());
    BUTTON_6.onTrue(pivot.testArm());
    BUTTON_7.onTrue(pivot.testArm1());
    BUTTON_8.onTrue(pivot.testArm2());
    BUTTON_12.whileTrue(new InstantCommand(climber::moveHooksUp));
    BUTTON_12.whileFalse(new InstantCommand(climber::stopHooks));
    BUTTON_13.whileTrue(new InstantCommand(climber::moveHooksDown));
    BUTTON_13.whileFalse(new InstantCommand(climber::stopHooks));
    b.whileTrue(new InstantCommand(indexer::ejectSpeaker));
    b.whileFalse(new InstantCommand(indexer::stopIndex));
    a.whileTrue(new EjectAmp());
    a.whileFalse(new InstantCommand(indexer::stopIndex));
  }

  public Command getAutonomousCommand() {
    return autos.getAutos();
  }
}
