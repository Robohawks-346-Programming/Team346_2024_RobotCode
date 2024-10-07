package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase{
    TalonFX topRoller, bottomRoller;
    private final VelocityVoltage voltage;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    private final VoltageOut volts;

    private double x, y;

    private final CoastOut coast = new CoastOut();

    private DigitalInput laserBreak;

    public Shooter() {
        topRoller = new TalonFX(Constants.ShooterConstants.TOP_SPEAKER_ROLLER_MOTOR_ID);
        bottomRoller = new TalonFX(Constants.ShooterConstants.BOTTOM_SPEAKER_ROLLER_MOTOR_ID);

        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterConfig.Slot0.kP = Constants.ShooterConstants.SPEAKER_SHOOTER_P;
        shooterConfig.Slot0.kI = Constants.ShooterConstants.SPEAKER_SHOOTER_I;
        shooterConfig.Slot0.kD = Constants.ShooterConstants.SPEAKER_SHOOTER_D;

        shooterConfig.Slot0.kV = Constants.ShooterConstants.SPEAKER_SHOOTER_kV;

        bottomRoller.getConfigurator().apply(shooterConfig);
        topRoller.getConfigurator().apply(shooterConfig);

        laserBreak = new DigitalInput(Constants.ShooterConstants.BEAM_BREAK_PORT);

        voltage = new VelocityVoltage(0);

        volts = new VoltageOut(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Roller RPM", topRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Roller RPM", bottomRoller.getVelocity().getValueAsDouble());
    }

    /**
     * Sets a specified velocity to the motors
     * @param velocity Velocity for the top roller
     * @param velocity2 Velocity for the bottom roller
     */
    public void setVelocity(double velocity, double velocity2) {
        topRoller.setControl(voltage.withVelocity(velocity));
        bottomRoller.setControl(voltage.withVelocity(velocity2));
    }

    public void setVoltage(double volt) {
        topRoller.setControl(volts.withOutput(volt));
        bottomRoller.setControl(volts.withOutput(volt));
    }

    public boolean isAtVelocity(double rev){
        return Math.abs(topRoller.getVelocity().getValueAsDouble() - rev) < 30;
    }

    public boolean getLaserBreak() {
        return !laserBreak.get();
    }
    
    public void stopShooter() {
        topRoller.setControl(coast);
        bottomRoller.setControl(coast);
    }
}