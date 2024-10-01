package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.*;;

public class RotateToHeading extends Command {
  PIDController m_turnController;
  double turnSpeed;
  Drivetrain drivetrain;

  public RotateToHeading() {
    drivetrain = RobotContainer.drivetrain;
    addRequirements(drivetrain);
    m_turnController = new PIDController(0.09, 0, 0);
    m_turnController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    drivetrain.drive(
        new ChassisSpeeds(
            0,
            0,
            //drivetrain.getAutomaticRotationSide()));
            m_turnController.calculate(drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), drivetrain.getHeadingAngleToSpeaker() - 2)));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("Finished Rotation", drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    return Math.abs(drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees() - drivetrain.getHeadingAngleToSpeaker()) <= 3;
  }

}