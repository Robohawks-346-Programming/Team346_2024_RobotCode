package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase{
    CANSparkMax leftHook, rightHook;
    SparkPIDController leftHookPIDController, rightHookPIDController;
    RelativeEncoder leftHookEncoder, rightHookEncoder;
    boolean leftHookGood;
    boolean rightHookGood;

    public Climber() {
        leftHook = new CANSparkMax(Constants.ClimberConstants.LEFT_HOOK_MOTOR_ID, MotorType.kBrushless);
        rightHook = new CANSparkMax(Constants.ClimberConstants.RIGHT_HOOK_MOTOR_ID, MotorType.kBrushless);

        leftHookPIDController = leftHook.getPIDController();
        rightHookPIDController = rightHook.getPIDController();

        leftHookEncoder = leftHook.getEncoder();
        rightHookEncoder = rightHook.getEncoder();

        leftHookPIDController.setP(Constants.ClimberConstants.HOOK_P);
        leftHookPIDController.setI(Constants.ClimberConstants.HOOK_I);
        leftHookPIDController.setD(Constants.ClimberConstants.HOOK_D);

        rightHookPIDController.setP(Constants.ClimberConstants.HOOK_P);
        rightHookPIDController.setI(Constants.ClimberConstants.HOOK_I);
        rightHookPIDController.setD(Constants.ClimberConstants.HOOK_D);

        leftHook.burnFlash();
        rightHook.burnFlash();

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Left hook Good", leftHookGood);
        SmartDashboard.putBoolean("Right hook Good", rightHookGood);

        SmartDashboard.putNumber("Left hook rev", leftHookEncoder.getVelocity());
        SmartDashboard.putNumber("Right hook rev", rightHookEncoder.getVelocity());
    }

    public void moveHooks(double setpoint) {
        leftHookPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        rightHookPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    public boolean getHooksGood(double setpoint) {
        if(Math.abs(leftHookEncoder.getPosition() - setpoint) <= Constants.ClimberConstants.CLIMBER_REV_THRESHOLD) {
            leftHookGood = true;
        }
        else {
            leftHookGood = false;
        }

        if(Math.abs(rightHookEncoder.getPosition() - setpoint) <= Constants.ClimberConstants.CLIMBER_REV_THRESHOLD){
            rightHookGood = true;
        }
        else {
            rightHookGood = false;
        }

        return leftHookGood && rightHookGood;
    }
}