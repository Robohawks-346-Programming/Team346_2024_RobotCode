package frc.robot.commands;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {
    Drivetrain m_drive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  double curXSpeed;
  double curYSpeed;
  double curZRotation;
  double deadzone;
  ChassisSpeeds speeds;
  private boolean isInverted = false;


  public TeleopDrive(Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation, double deadzone, boolean inverted) {
    m_drive = RobotContainer.drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    this.deadzone = deadzone;
    isInverted = inverted;
    addRequirements(RobotContainer.drivetrain);
    // if(isInverted)  {
    //   curXSpeed *= -1;
    //   curYSpeed *= -1; 
    // }
  }

  public void execute() {
    
    
    curXSpeed = Math.abs(xSpeed.get()) < deadzone ? 0 : xSpeed.get();
    curYSpeed = Math.abs(ySpeed.get()) < deadzone ? 0 : ySpeed.get();
    curZRotation = Math.abs(zRotation.get()) < deadzone ? 0 : zRotation.get();
    curXSpeed *= DriveConstants.MAX_MOVE_VELOCITY;
    curYSpeed *= DriveConstants.MAX_MOVE_VELOCITY;
    curZRotation *= DriveConstants.MAX_TURN_VELOCITY;
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){

      curXSpeed *=-1;
      curYSpeed *=-1;
    }

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(curXSpeed, curYSpeed, curZRotation,
        m_drive.getEstimatorPose().getRotation());
    m_drive.drive(speeds);
  }
}