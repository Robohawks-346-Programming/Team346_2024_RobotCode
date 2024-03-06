package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {

  Drivetrain drivetrain;
  DoubleSupplier x,y,theta;
  double moveVelocity, turnVelocity;
  boolean speedBoost, isInverted;

  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean speedBoost, boolean isInverted) {
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.speedBoost = speedBoost;
    this.isInverted = isInverted;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(speedBoost) {
      if(isInverted) {
        moveVelocity = -Constants.DriveConstants.MAX_MOVE_VELOCITY_FAST;
        turnVelocity = -Constants.DriveConstants.MAX_TURN_VELOCITY_FAST;
      }
      else {
        moveVelocity = Constants.DriveConstants.MAX_MOVE_VELOCITY_FAST;
        turnVelocity = Constants.DriveConstants.MAX_TURN_VELOCITY_FAST;
      }
    }

    else {
      if(isInverted) {
        moveVelocity = -Constants.DriveConstants.MAX_MOVE_VELOCITY;
        turnVelocity = -Constants.DriveConstants.MAX_TURN_VELOCITY;
      }
      else {
        moveVelocity = Constants.DriveConstants.MAX_MOVE_VELOCITY;
        turnVelocity = Constants.DriveConstants.MAX_TURN_VELOCITY;
      }
    }
    
    double doubleX = Math.abs(x.getAsDouble()) < 0.07 ? 0 : x.getAsDouble();
    double doubleY = Math.abs(y.getAsDouble()) < 0.07 ? 0 : y.getAsDouble();
    double doubleTheta = Math.abs(theta.getAsDouble()) < 0.07 ? 0 : theta.getAsDouble();

    double vx = doubleX * moveVelocity;
    double vy = doubleY * moveVelocity;
    double omega = doubleTheta * turnVelocity;

    ChassisSpeeds velocity = Constants.DriveConstants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.getHeading()) 
      : new ChassisSpeeds(vx, vy, omega);

    drivetrain.drive(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}