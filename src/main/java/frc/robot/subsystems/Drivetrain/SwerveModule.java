package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

   CANSparkMax driveMotor, turnMotor;

   RelativeEncoder driveEncoder, turnEncoder;

   CANCoder turningCANCoder;

   double encoderOffset;

   SparkMaxPIDController driveController, turnController;

   private double adjustedSpeed;

   public SwerveModule (
    int driveMotorID,
    int turnMotorID,
    int turningCANCoderID,
    double turnEncoderOffset) 
    {

        encoderOffset = turnEncoderOffset;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        turningCANCoder = new CANCoder(turningCANCoderID);
        turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turningCANCoder.configSensorDirection(false);

        driveEncoder.setVelocityConversionFactor(Constants.DriveConstants.DRIVE_CONVERSION / 60);
        driveEncoder.setPositionConversionFactor(Constants.DriveConstants.DRIVE_CONVERSION);

        turnEncoder.setPositionConversionFactor(360.0 / Constants.DriveConstants.TURN_CONVERSION);

        driveMotor.setInverted(true);
        turnMotor.setInverted(true);
        
        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.enableVoltageCompensation(Constants.DriveConstants.MAX_VOLTAGE);
        turnMotor.enableVoltageCompensation(Constants.DriveConstants.MAX_VOLTAGE);

        driveMotor.setSmartCurrentLimit(Constants.DriveConstants.DRIVE_CURRENT_LIMIT);
        turnMotor.setSmartCurrentLimit(Constants.DriveConstants.TURN_CURRENT_LIMIT);

        driveController = driveMotor.getPIDController();
        turnController = turnMotor.getPIDController();

        turnController.setPositionPIDWrappingEnabled(true);
        turnController.setPositionPIDWrappingMinInput(-180);
        turnController.setPositionPIDWrappingMaxInput(180);

        driveController.setP(Constants.DriveConstants.DRIVE_P);
        driveController.setI(Constants.DriveConstants.DRIVE_I);
        driveController.setD(Constants.DriveConstants.DRIVE_D);
        //driveController.setFF(Constants.DriveConstants.DRIVE_FF);

        turnController.setP(Constants.DriveConstants.TURN_P);
        turnController.setI(Constants.DriveConstants.TURN_I);
        turnController.setD(Constants.DriveConstants.TURN_D);
        turnController.setFF(Constants.DriveConstants.TURN_FF);

        driveMotor.burnFlash();
        turnMotor.burnFlash();  
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getStateAngle());
    }

    public SwerveModuleState getAbsoluteState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteRotation());
    }

    public Rotation2d getStateAngle() {
        double stateAngle = Units.degreesToRadians(turnEncoder.getPosition());
        return new Rotation2d(MathUtil.angleModulus(stateAngle));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromDegrees(turningCANCoder.getAbsolutePosition());
    }

    public Rotation2d adjustedAngle = new Rotation2d();

    public void setState(SwerveModuleState state) {
        double driveOutput = state.speedMetersPerSecond;
        SmartDashboard.putNumber("Velocity Input", driveOutput);
        turnController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        adjustedSpeed = driveOutput;
        driveController.setReference(driveOutput, ControlType.kVelocity, 0, 2.35 * adjustedSpeed);
    }

    public void setAutoState(SwerveModuleState state) {
        double driveOutput = state.speedMetersPerSecond;
        SmartDashboard.putNumber("Velocity Input", driveOutput);
        turnController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        adjustedSpeed = driveOutput;
        driveController.setReference(-driveOutput, ControlType.kVelocity, 0, 2.35 * adjustedSpeed);
    }

    public double adjustedAngle(double wantedAngle, double currentAngle) {
        return ((wantedAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistance() {
        return driveEncoder.getPosition();
    }

    public void resetDistance() {
        driveEncoder.setPosition(0.0);
    }

    public void syncTurnEncoders() {
        turnEncoder.setPosition(turningCANCoder.getAbsolutePosition());
    }

    public void resetEncoders() {
        turnEncoder.setPosition(turningCANCoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getStateAngle());
    }

    public void resetAbsoluteEncoder() {
        turningCANCoder.setPosition(0);
        turningCANCoder.setPositionToAbsolute();
        turningCANCoder.configMagnetOffset(turningCANCoder.configGetMagnetOffset()- turningCANCoder.getAbsolutePosition());
    }

    public Rotation2d getAngle() {
        double angle = Units.degreesToRadians(turnEncoder.getPosition());
        return new Rotation2d(MathUtil.angleModulus(angle));
    }

    public double turnAngleRadians() {
        return encoderOffset + (turnEncoder.getPosition() * 2 * Math.PI); 
    }

    public double turnAngleDegrees() {
        return encoderOffset + Math.toDegrees(turnEncoder.getPosition() * 2 * Math.PI); 
    }

        public double getMetersDriven() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (driveEncoder.getPosition());
    }
}