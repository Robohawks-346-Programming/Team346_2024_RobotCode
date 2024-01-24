// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;


// public class Arm extends SubsystemBase{
//     CANSparkMax topSpeakerRoller, bottomSpeakerRoller, feederRoller, ampRollers;
//     SparkPIDController topSpeakerRollerPIDController, bottomSpeakerRollerPIDController;
//     RelativeEncoder topSpeakerRollerEncoder, bottomSpeakerRollerEncoder;
//     private DigitalInput laserBreak;
//     boolean shooterVelocity;

//     double setPoint;
//     public Arm() {
//         topSpeakerRoller = new CANSparkMax(Constants.ShooterConstants.TOP_SPEAKER_ROLLER_MOTOR_ID, MotorType.kBrushless);
//         bottomSpeakerRoller = new CANSparkMax(Constants.ShooterConstants.BOTTOM_SPEAKER_ROLLER_MOTOR_ID, MotorType.kBrushless);
//         feederRoller = new CANSparkMax(Constants.ShooterConstants.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
//         ampRollers = new CANSparkMax(Constants.ShooterConstants.AMP_ROLLER_MOTOR_ID, MotorType.kBrushless);

//         laserBreak = new DigitalInput(Constants.ShooterConstants.INTAKE_LASER_BREAK_PORT);

//         topSpeakerRollerPIDController = topSpeakerRoller.getPIDController();
//         bottomSpeakerRollerPIDController = bottomSpeakerRoller.getPIDController();

//         topSpeakerRollerEncoder = topSpeakerRoller.getEncoder();
//         bottomSpeakerRollerEncoder = bottomSpeakerRoller.getEncoder();

//         topSpeakerRollerPIDController.setP(Constants.ShooterConstants.SPEAKER_SHOOTER_TOP_P);
//         topSpeakerRollerPIDController.setI(Constants.ShooterConstants.SPEAKER_SHOOTER_TOP_I);
//         topSpeakerRollerPIDController.setD(Constants.ShooterConstants.SPEAKER_SHOOTER_TOP_D);

//         bottomSpeakerRollerPIDController.setP(Constants.ShooterConstants.SPEAKER_SHOOTER_BOTTOM_P);
//         bottomSpeakerRollerPIDController.setI(Constants.ShooterConstants.SPEAKER_SHOOTER_BOTTOM_I);
//         bottomSpeakerRollerPIDController.setD(Constants.ShooterConstants.SPEAKER_SHOOTER_BOTTOM_D);

//         topSpeakerRoller.burnFlash();
//         bottomSpeakerRoller.burnFlash();

//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putBoolean("Intake laser", getLaserBreak());
//         SmartDashboard.putNumber("Top Roller RPM", topSpeakerRollerEncoder.getVelocity());
//         SmartDashboard.putNumber("Bottom Roller RPM", bottomSpeakerRollerEncoder.getVelocity());
//         SmartDashboard.putBoolean("Shooter is Good", shooterVelocity); //Tells when ready to shoot
//         SmartDashboard.putNumber("Shooter RPM", setPoint);
//     }

//     public void setSpeakerShooterSpeed(double shooterSpeed) {
//         topSpeakerRoller.set(shooterSpeed);
//         bottomSpeakerRoller.set(shooterSpeed);
//     }

//     public void setSpeakerShooterVelocity(double setPoint) {
//         topSpeakerRollerPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
//         bottomSpeakerRollerPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
//         if(Math.abs(topSpeakerRollerEncoder.getVelocity() - setPoint) <= Constants.ShooterConstants.SPEAKER_ROLLER_VELOCITY_THRESHOLD && Math.abs(bottomSpeakerRollerEncoder.getVelocity() - setPoint) <= Constants.ShooterConstants.SPEAKER_ROLLER_VELOCITY_THRESHOLD) {
//             shooterVelocity = true;
//         }
//         else {
//             shooterVelocity = false;
//         }
//     }

//     public boolean getSpeakerShooterVelocity(double setpoint){
//         return setpoint == Math.min(topSpeakerRollerEncoder.getVelocity(), bottomSpeakerRollerEncoder.getVelocity());
//     }

//     public boolean getLaserBreak() {
//         return laserBreak.get();
//     }

//     public void armIntake() {
//         feederRoller.set(Constants.ShooterConstants.FEEDER_ROLLER_SPEED);
//         ampRollers.set(Constants.ShooterConstants.AMP_ROLLERS_ROLLER_SPEED);
//     }

//     public void stopArmIntake() {
//         feederRoller.set(0);
//         ampRollers.set(0);
//     }

//     public void ejectSpeaker() {
//         feederRoller.set(Constants.ShooterConstants.FEEDER_ROLLER_SPEED);
//     }

//     public void ejectAmp() {
//         feederRoller.set(-Constants.ShooterConstants.FEEDER_ROLLER_SPEED);
//         ampRollers.set(Constants.ShooterConstants.AMP_ROLLERS_ROLLER_SPEED);
//     }

//     public void stopAmpShooter() {
//         feederRoller.set(0);
//         ampRollers.set(0);
//     }
// }