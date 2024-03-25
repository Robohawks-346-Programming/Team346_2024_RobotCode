package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Indexer extends SubsystemBase{
    CANSparkMax feederRoller, ampRollers;

    public boolean storingGamePiece;

    public final Timer timer = new Timer();

    public Indexer() {
        feederRoller = new CANSparkMax(Constants.IndexerConstants.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
        ampRollers = new CANSparkMax(Constants.IndexerConstants.AMP_ROLLER_MOTOR_ID, MotorType.kBrushless);

        feederRoller.setSmartCurrentLimit(40);
        ampRollers.setSmartCurrentLimit(40);

        feederRoller.setIdleMode(IdleMode.kBrake);
        ampRollers.setIdleMode(IdleMode.kBrake);

        feederRoller.setInverted(true);
        ampRollers.setInverted(false);

        feederRoller.burnFlash();
        ampRollers.burnFlash();

        timer.reset();
        timer.start();
    }

    public void startIndex() {
        feederRoller.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_1);
        ampRollers.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_1);
    }

    public void reverseIndex() {
        feederRoller.set(-Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_2);
        ampRollers.set(-Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_2);
    }

    public void stopIndex() {
        feederRoller.set(0);
        ampRollers.set(0);
    }

    public void ejectAmp() {
        feederRoller.set(-Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_1);
        ampRollers.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_2);
    }

    public void stopAmp() {
        feederRoller.set(0);
        ampRollers.set(0);
    }

    public boolean returnCurrent() {
        //return (feederRoller.getOutputCurrent() > 15) && timer.hasElapsed(1);
        return feederRoller.getOutputCurrent() >15;
    }

    public void toggleStoringGamepiece() {
        storingGamePiece = !storingGamePiece;
    }

    public void ejectSpeaker() {
        feederRoller.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_2);
        ampRollers.set(Constants.IndexerConstants.AMP_ROLLERS_ROLLER_SPEED_2);
    }

    public void indexSlow(){
        feederRoller.set(0.25);
        ampRollers.set(0.25);
    }
}