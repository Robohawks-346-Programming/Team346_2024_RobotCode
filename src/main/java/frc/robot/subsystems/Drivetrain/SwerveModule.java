package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule extends SubsystemBase {

   //CANSparkMax turnMotor;
   CANSparkMax driveMotor;

   TalonFX turnMotor;
   private final PositionVoltage anglePosition;

   RelativeEncoder driveEncoder;
   //RelativeEncoder turnEncoder;

   CANcoder turningCANCoder;
   MagnetSensorConfigs magnetSensorConfigs;

   Rotation2d encoderOffset;

   SparkPIDController driveController;
   
   //SparkPIDController turnController;

   private double adjustedSpeed;

   public SwerveModule (
    int driveMotorID,
    int turnMotorID,
    int turningCANCoderID,
    Rotation2d turnEncoderOffset, boolean invert) 
    {
        encoderOffset = turnEncoderOffset;
        //turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        turningCANCoder = new CANcoder(turningCANCoderID);
        turnMotor = new TalonFX(turnMotorID);

        CTREConfigs configs = new CTREConfigs();

        turningCANCoder.getConfigurator().apply(configs.swerveCANcoderConfig); 
        turnMotor.getConfigurator().apply(configs.swerveAngleFXConfig);  

        anglePosition = new PositionVoltage(0);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        //turnMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        //turnEncoder = turnMotor.getEncoder();

        // turningCANCoder = new CANcoder(turningCANCoderID);
        // magnetSensorConfigs = new MagnetSensorConfigs();
        // turningCANCoder.getConfigurator().apply(magnetSensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf));

        driveEncoder.setVelocityConversionFactor(Constants.DriveConstants.DRIVE_CONVERSION / 60);
        driveEncoder.setPositionConversionFactor(Constants.DriveConstants.DRIVE_CONVERSION);

        //turnEncoder.setPositionConversionFactor(360.0 / Constants.DriveConstants.TURN_CONVERSION);

        driveMotor.setInverted(invert);
        turnMotor.setInverted(false);
        
        driveMotor.setIdleMode(IdleMode.kCoast);
        //turnMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.enableVoltageCompensation(Constants.DriveConstants.MAX_VOLTAGE);
        //turnMotor.enableVoltageCompensation(Constants.DriveConstants.MAX_VOLTAGE);

        driveMotor.setSmartCurrentLimit(Constants.DriveConstants.DRIVE_CURRENT_LIMIT);
        //turnMotor.setSmartCurrentLimit(Constants.DriveConstants.TURN_CURRENT_LIMIT);

        driveController = driveMotor.getPIDController();
        //turnController = turnMotor.getPIDController();

        // turnController.setPositionPIDWrappingEnabled(true);
        // turnController.setPositionPIDWrappingMinInput(-180);
        // turnController.setPositionPIDWrappingMaxInput(180);

        driveController.setP(Constants.DriveConstants.DRIVE_P);
        driveController.setI(Constants.DriveConstants.DRIVE_I);
        driveController.setD(Constants.DriveConstants.DRIVE_D);
        //driveController.setFF(Constants.DriveConstants.DRIVE_FF);

        // turnController.setP(Constants.DriveConstants.TURN_P);
        // turnController.setI(Constants.DriveConstants.TURN_I);
        // turnController.setD(Constants.DriveConstants.TURN_D);
        // turnController.setFF(Constants.DriveConstants.TURN_FF);

        driveMotor.burnFlash();
        //turnMotor.burnFlash();  

        resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), 
        Rotation2d.fromRotations(turnMotor.getPosition().getValue()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        double driveOutput = state.speedMetersPerSecond;
        SmartDashboard.putNumber("Velocity Input", driveOutput);
        //turnController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        turnMotor.setControl(anglePosition.withPosition(state.angle.getRotations()));
        adjustedSpeed = driveOutput;
        driveController.setReference(driveOutput, ControlType.kVelocity, 0, 2.35 * adjustedSpeed);
    }

    public void resetDistance() {
        driveEncoder.setPosition(0.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), 
        Rotation2d.fromRotations(turnMotor.getPosition().getValue()));
    }

    public void resetAbsoluteEncoder() {
        turningCANCoder.setPosition(0);
    }

    public double getMetersDriven() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (driveEncoder.getPosition());
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

}