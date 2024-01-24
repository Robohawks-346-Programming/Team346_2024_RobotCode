package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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
   TalonFXConfiguration swerveAngleFXConfig;
   CANcoderConfiguration swerveCANcoderConfig;
   private final PositionVoltage anglePosition;

   RelativeEncoder driveEncoder;
   //RelativeEncoder turnEncoder;

   CANcoder turningCANCoder;
   MagnetSensorConfigs magnetSensorConfigs;

   double encoderOffset;

   SparkPIDController driveController;
   //SparkPIDController turnController;

   private double adjustedSpeed;

   public SwerveModule (
    int driveMotorID,
    int turnMotorID,
    int turningCANCoderID,
    double turnEncoderOffset) 
    {
        //turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor = new TalonFX(turnMotorID);
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveCANcoderConfig = new CANcoderConfiguration();

        swerveCANcoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        turningCANCoder = new CANcoder(turningCANCoderID);
        turningCANCoder.getConfigurator().apply(swerveCANcoderConfig);

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        swerveAngleFXConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.withSensorToMechanismRatio(12.8 / 1.0);
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
<<<<<<< Updated upstream
        swerveAngleFXConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
        swerveAngleFXConfig.CurrentLimits.withSupplyCurrentLimit(25);
        swerveAngleFXConfig.CurrentLimits.withSupplyCurrentThreshold(40);
        swerveAngleFXConfig.CurrentLimits.withSupplyTimeThreshold(0.1);
=======
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = 25;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
>>>>>>> Stashed changes

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.DriveConstants.TURN_P;
        swerveAngleFXConfig.Slot0.kI = Constants.DriveConstants.TURN_I;
        swerveAngleFXConfig.Slot0.kD = Constants.DriveConstants.TURN_D;

        turnMotor.getConfigurator().apply(swerveAngleFXConfig);

        anglePosition = new PositionVoltage(0);

        encoderOffset = turnEncoderOffset;

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

        driveMotor.setInverted(true);
        turnMotor.setInverted(true);
        
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
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getStateAngle());
    }

    public SwerveModuleState getAbsoluteState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteRotation());
    }

    public Rotation2d getStateAngle() {
        double stateAngle = Units.degreesToRadians(turnMotor.getPosition().getValue());
        return new Rotation2d(MathUtil.angleModulus(stateAngle));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAbsoluteRotation() {
        //return Rotation2d.fromDegrees(turningCANCoder.getAbsolutePosition());
        return Rotation2d.fromRotations(turningCANCoder.getAbsolutePosition().getValue());
    }

    public Rotation2d adjustedAngle = new Rotation2d();

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle); 
        double driveOutput = state.speedMetersPerSecond;
        SmartDashboard.putNumber("Velocity Input", driveOutput);
        //turnController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        turnMotor.setControl(anglePosition.withPosition(state.angle.getRotations()));
        adjustedSpeed = driveOutput;
        driveController.setReference(driveOutput, ControlType.kVelocity, 0, 2.35 * adjustedSpeed);
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
        turnMotor.setPosition(turningCANCoder.getAbsolutePosition().getValue());
    }

    public void resetEncoders() {
        turnMotor.setPosition(turningCANCoder.getAbsolutePosition().getValue());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getStateAngle());
    }

    public void resetAbsoluteEncoder() {
        turningCANCoder.setPosition(0);
    }

    public Rotation2d getAngle() {
        double angle = Units.degreesToRadians(turnMotor.getPosition().getValue());
        return new Rotation2d(MathUtil.angleModulus(angle));
    }

    public double turnAngleRadians() {
        return encoderOffset + (turnMotor.getPosition().getValue() * 2 * Math.PI); 
    }

    public double turnAngleDegrees() {
        return encoderOffset + Math.toDegrees(turnMotor.getPosition().getValue() * 2 * Math.PI); 
    }

        public double getMetersDriven() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (driveEncoder.getPosition());
    }
}