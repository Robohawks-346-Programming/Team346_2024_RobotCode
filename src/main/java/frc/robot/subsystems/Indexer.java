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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.CTREConfigs;


public class Indexer extends SubsystemBase{
    CANSparkMax feederRoller, ampRollers;

    public Indexer() {
        feederRoller = new CANSparkMax(Constants.IndexerConstants.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
        ampRollers = new CANSparkMax(Constants.IndexerConstants.AMP_ROLLER_MOTOR_ID, MotorType.kBrushless);

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
}