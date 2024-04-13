package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.subsystems.Vision.Vision;;

public class AutoLockNote extends Command {
  PIDController m_driveController;
  double turnSpeed;
  Drivetrain drivetrain;
  Vision vision;
  double wantedX, wantedY;

  public AutoLockNote() {
    drivetrain = RobotContainer.drivetrain;
    vision = RobotContainer.vision;
    addRequirements(drivetrain);
    m_driveController = new PIDController(0.15, 0, 0);
    m_driveController.enableContinuousInput(-10, 10);
    wantedX = drivetrain.poseEstimator.getEstimatedPosition().getX() + vision.getNoteX();
    wantedY = drivetrain.poseEstimator.getEstimatedPosition().getY() + vision.getNoteY();
  }

  @Override
  public void execute() {
    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_driveController.calculate(drivetrain.poseEstimator.getEstimatedPosition().getX(), wantedX),
            m_driveController.calculate(drivetrain.poseEstimator.getEstimatedPosition().getY(), wantedY),
            0,
            drivetrain.getHeading()));
  }
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
  }

  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("Finished Rotation", drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    return (Math.abs(drivetrain.poseEstimator.getEstimatedPosition().getX() - wantedX) <= 0.1) &&
    (Math.abs(drivetrain.poseEstimator.getEstimatedPosition().getY() - wantedY) <= 0.1);
  }

}