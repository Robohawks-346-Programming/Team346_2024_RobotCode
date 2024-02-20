package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.CTREConfigs;


public class Indexer extends SubsystemBase{
    CANSparkMax feederRoller, ampRollers;

    public final Timer timer = new Timer();

    public Indexer() {
        feederRoller = new CANSparkMax(Constants.IndexerConstants.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
        ampRollers = new CANSparkMax(Constants.IndexerConstants.AMP_ROLLER_MOTOR_ID, MotorType.kBrushless);

        feederRoller.setSmartCurrentLimit(40);
        ampRollers.setSmartCurrentLimit(40);

        feederRoller.setInverted(true);
        ampRollers.setInverted(false);

        feederRoller.burnFlash();
        ampRollers.burnFlash();

        timer.reset();
        timer.start();
    }

    public void startIndex() {
        feederRoller.set(Constants.IndexerConstants.FEEDER_ROLLER_SPEED);
        ampRollers.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED);
    }

    public void stopIndex() {
        feederRoller.set(0);
        ampRollers.set(0);
    }

    public void ejectAmp() {
        feederRoller.set(-Constants.IndexerConstants.FEEDER_ROLLER_SPEED);
        ampRollers.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED);
    }

    public void stopAmp() {
        feederRoller.set(0);
        ampRollers.set(0);
    }

    public boolean returnCurrent() {
        return (feederRoller.getOutputCurrent() > 20) && timer.hasElapsed(0.7);
    }
}