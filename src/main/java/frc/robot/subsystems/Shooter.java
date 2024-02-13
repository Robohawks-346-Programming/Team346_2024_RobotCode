package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.CTREConfigs;


public class Shooter extends SubsystemBase{
    TalonFX topRoller, bottomRoller;
    private final VelocityVoltage voltage;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    public Shooter() {
        topRoller = new TalonFX(Constants.ShooterConstants.TOP_SPEAKER_ROLLER_MOTOR_ID);
        bottomRoller = new TalonFX(Constants.ShooterConstants.BOTTOM_SPEAKER_ROLLER_MOTOR_ID);

        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterConfig.Slot0.kP = Constants.ShooterConstants.SPEAKER_SHOOTER_P;
        shooterConfig.Slot0.kI = Constants.ShooterConstants.SPEAKER_SHOOTER_I;
        shooterConfig.Slot0.kD = Constants.ShooterConstants.SPEAKER_SHOOTER_D;

        bottomRoller.getConfigurator().apply(shooterConfig);
        topRoller.getConfigurator().apply(shooterConfig);

        voltage = new VelocityVoltage(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Roller RPM", topRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Roller RPM", bottomRoller.getVelocity().getValueAsDouble());
    }

    public void setVelocity(double velocity) {
        topRoller.setControl(voltage.withVelocity(velocity));
        bottomRoller.setControl(voltage.withVelocity(velocity));
    }
}