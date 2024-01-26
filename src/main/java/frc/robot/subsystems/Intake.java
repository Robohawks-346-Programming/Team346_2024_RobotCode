package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static CANSparkMax intakeMotor;
    
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(25);
    }

    // Runs Intake Motor
    public void runIntake() {
        intakeMotor.set(Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
        RobotState.getInstance().setIntaking(true);
    }

    // Runs Intake Motor
    public void runOutake() {
        intakeMotor.set(-Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
    }
    
    // Stops Intake Motor when finished
    public void stopIntake() {
        intakeMotor.set(0.0);
    }
}