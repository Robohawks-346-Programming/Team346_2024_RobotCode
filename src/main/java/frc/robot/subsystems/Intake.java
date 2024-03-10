package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static CANSparkMax intakeMotor;
    private static CANSparkMax centeringMotor;

    public boolean isIntaking, hasGamePiece = false;

    public final Timer timer = new Timer();
    
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        centeringMotor = new CANSparkMax(Constants.IntakeConstants.CENTERING_MOTOR_ID, MotorType.kBrushless);

        intakeMotor.setSmartCurrentLimit(40);
        centeringMotor.setSmartCurrentLimit(40);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        centeringMotor.setIdleMode(IdleMode.kBrake);

        centeringMotor.setInverted(false);

        intakeMotor.burnFlash();
        centeringMotor.burnFlash();

        timer.reset();
        timer.start();
    }

    // Runs Intake Motor
    public void runIntake() {
        intakeMotor.set(Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
        centeringMotor.set(Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    // Runs Intake Motor
    public void runOutake() {
        intakeMotor.set(-Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
        centeringMotor.set(-Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
    }
    
    // Stops Intake Motor when finished
    public void stopIntake() {
        intakeMotor.set(0);
        centeringMotor.set(0);
    }

    public boolean getCurrent() {
        return (centeringMotor.getOutputCurrent() > 20) && timer.hasElapsed(1);
    }

    public void toggleIsIntaking() {
        isIntaking = !isIntaking;
    }

    public void toggleHasGamePiece() {
        hasGamePiece = !hasGamePiece;
    }

    public boolean returnCurrent() {
        return (intakeMotor.getOutputCurrent() > 20) && timer.hasElapsed(1);
    }
}