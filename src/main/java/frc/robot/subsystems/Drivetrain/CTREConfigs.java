package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = (12.8/1.0);
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.DriveConstants.TURN_P;
        swerveAngleFXConfig.Slot0.kI = Constants.DriveConstants.TURN_I;
        swerveAngleFXConfig.Slot0.kD = Constants.DriveConstants.TURN_D;
    }
}