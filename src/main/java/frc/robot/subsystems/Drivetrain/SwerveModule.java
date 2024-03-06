package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
   TalonFX turnMotor, driveMotor;
   private final PositionVoltage anglePosition;
   private final VelocityVoltage driveVelocity;
   private final VoltageOut volts;

   CANcoder turningCANCoder;
   Rotation2d encoderOffset;
   private double adjustedSpeed;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.DriveConstants.DRIVE_kS, Constants.DriveConstants.DRIVE_kV, Constants.DriveConstants.DRIVE_kA);


   public SwerveModule (
    int driveMotorID,
    int turnMotorID,
    int turningCANCoderID,
    Rotation2d turnEncoderOffset, boolean invert) 
    {
        encoderOffset = turnEncoderOffset;

        volts = new VoltageOut(0);

        turningCANCoder = new CANcoder(turningCANCoderID);
        turnMotor = new TalonFX(turnMotorID);
        driveMotor = new TalonFX(driveMotorID);

        CTREConfigs configs = new CTREConfigs();

        turningCANCoder.getConfigurator().apply(configs.swerveCANcoderConfig); 
        turnMotor.getConfigurator().apply(configs.swerveAngleFXConfig);  
        driveMotor.getConfigurator().apply(configs.swerveDriveFXConfig);

        anglePosition = new PositionVoltage(0);
        driveVelocity = new VelocityVoltage(0);

        resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE, 
        Rotation2d.fromRotations(turnMotor.getPosition().getValue()));
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE;
    }

    public void setState(SwerveModuleState state) {
        SmartDashboard.putNumber("Velocity Input", state.speedMetersPerSecond);
        state = SwerveModuleState.optimize(state, getState().angle); 
        turnMotor.setControl(anglePosition.withPosition(state.angle.getRotations()));
        setSpeed(state);
    }

    private void setSpeed(SwerveModuleState desiredState){
            driveVelocity.Velocity = desiredState.speedMetersPerSecond / Constants.DriveConstants.WHEEL_CIRCUMFERENCE;
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            driveMotor.setControl(driveVelocity);
    }

    public void resetDistance() {
        driveMotor.setPosition(0.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE, 
        Rotation2d.fromRotations(turnMotor.getPosition().getValue()));
    }

    public void resetAbsoluteEncoder() {
        turningCANCoder.setPosition(0);
    }

    public double getMetersDriven() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (driveMotor.getPosition().getValueAsDouble()) * Constants.DriveConstants.WHEEL_CIRCUMFERENCE;
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(turningCANCoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - encoderOffset.getRotations();
        turnMotor.setPosition(absolutePosition);
    }

    public double turnAngleDegrees() {
        return encoderOffset.getDegrees() + Math.toDegrees(turnMotor.getPosition().getValue() * 2 * Math.PI); 
    }

    public double canCoderRotations() {
        return turningCANCoder.getPosition().getValue();
    }

    public void resetEncoders() {
        turnMotor.setPosition(turningCANCoder.getPosition().getValue());
    }

    public void setDriveWheelsToVoltage(double volt) {
        driveMotor.setControl(volts.withOutput(volt));
    }

}