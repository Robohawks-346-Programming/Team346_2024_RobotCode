package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.IntakeFull;
import frc.robot.subsystems.Drivetrain.*;
import frc.robot.subsystems.Vision.Vision;;

public class AutoLockNote extends Command {
  PIDController m_driveController;
  Drivetrain drivetrain;
  Vision vision;
  double wantedX, wantedY;

  public AutoLockNote() {
    drivetrain = RobotContainer.drivetrain;
    vision = RobotContainer.vision;
    addRequirements(drivetrain);
    m_driveController = new PIDController(1.5, 0, 0);

    wantedX = drivetrain.poseEstimator.getEstimatedPosition().getX() + vision.getNoteX();
    wantedY = drivetrain.poseEstimator.getEstimatedPosition().getY() + vision.getNoteY();
  }

  @Override
  public void execute() {
    // noteX = vision.getNoteX();
    // noteY = vision.getNoteY();
    // wantedX = drivetrain.poseEstimator.getEstimatedPosition().getX() + noteX;
    // wantedY = drivetrain.poseEstimator.getEstimatedPosition().getY() + noteY;
    drivetrain.drive(
      new ChassisSpeeds(
            m_driveController.calculate(drivetrain.poseEstimator.getEstimatedPosition().getX(), wantedX),
            -m_driveController.calculate(drivetrain.poseEstimator.getEstimatedPosition().getY(), wantedY),
            0));
  }
  @Override
  public void end(boolean interrupted) {
    drivetrain.brake();
    vision.resetDistances();
  }

  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("Finished Rotation", drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    return ((Math.abs(drivetrain.poseEstimator.getEstimatedPosition().getX() - wantedX) <= 0.05) &&
    (Math.abs(drivetrain.poseEstimator.getEstimatedPosition().getY() - wantedY) <= 0.05));
  }

}