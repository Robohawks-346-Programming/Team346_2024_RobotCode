// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Pivot extends SubsystemBase {
//     private static CANSparkMax rotationMotor;
//     private static DoubleSolenoid brakeSolenoid;
//     private static RelativeEncoder rotationEncoder;
//     private static SparkPIDController rotationPIDController;

//     double armDegreesPerMotorRev;
    
//     public Pivot() {
//         rotationMotor = new CANSparkMax(Constants.PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);    
//         rotationMotor.setIdleMode(IdleMode.kBrake);
//         rotationEncoder = rotationMotor.getEncoder();
//         armDegreesPerMotorRev = 360/Constants.PivotConstants.PIVOT_GEAR_RATIO;
        
//         rotationEncoder.setPositionConversionFactor(armDegreesPerMotorRev);
//         rotationEncoder.setPosition(Constants.PivotConstants.HOME_PIVOT_ANGLE);
//         rotationMotor.setInverted(true);
//         rotationMotor.setIdleMode(IdleMode.kBrake);

//         rotationPIDController = rotationMotor.getPIDController();
//         rotationPIDController.setP(Constants.PivotConstants.PIVOT_P);
//         rotationPIDController.setI(Constants.PivotConstants.PIVOT_I);
//         rotationPIDController.setD(Constants.PivotConstants.PIVOT_D);
        

//         rotationPIDController.setOutputRange(-Constants.PivotConstants.PIVOT_MOTOR_SPEED_DOWN, Constants.PivotConstants.PIVOT_MOTOR_SPEED_UP);

//         rotationMotor.burnFlash();

//         brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PivotConstants.PIVOT_BRAKE_FORWARD_CHANNEL, Constants.PivotConstants.PIVOT_BRAKE_REVERSE_CHANNEL);

//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Arm Degrees", getRotationEncoder());
//     }

//     // Resetting Rotation encoders
//     public void resetRotationEncoder() {
//         rotationEncoder.setPosition(0.0);
//     }

//     public double getRotationEncoder() {
//         return rotationEncoder.getPosition();
//     }

//     public void setRotationEncoder() {
//         rotationEncoder.setPosition(Constants.PivotConstants.HOME_PIVOT_ANGLE);
//     }

//     // Checks to see if the position has been reached
//     public boolean isAtPosition(double rev) {
//         double difference = Math.abs(rotationEncoder.getPosition() - rev);
//         return(difference <= Constants.PivotConstants.PIVOT_ANGLE_THRESHOLD);
//     }

//     // Stops rotation motor once finished
//     public void stopRotationMotor() {
//         rotationMotor.set(0.0);
//     }

//     public double lerpSpeed(double aI, double aF, double bI, double bF) {
//         return (bI * (aF - getRotationEncoder()) + bF * (getRotationEncoder() - aI)) / (aF - aI);
//     }

//     public void moveArm(double wantedPosition, double currentDegree) {
//         double currentPosition = rotationEncoder.getPosition();
//         if (wantedPosition > currentPosition) {
//             rotationMotor.set(lerpSpeed(currentDegree, wantedPosition, Constants.PivotConstants.PIVOT_MOTOR_SPEED_UP, Constants.PivotConstants.PIVOT_MOTOR_SPEED_UP_FINAL));
//         }

//         else if (wantedPosition < currentPosition) {
//             rotationMotor.set(-lerpSpeed(currentDegree, wantedPosition, Constants.PivotConstants.PIVOT_MOTOR_SPEED_DOWN, Constants.PivotConstants.PIVOT_MOTOR_SPEED_DOWN_FINAL));
//         }

//         else {
//             rotationMotor.set(0.0);
//         }
//     }

//     public void moveArmToPosition(double wantedPosition) {
//         rotationPIDController.setReference(wantedPosition, ControlType.kPosition);
//     }

//     public void engageBrake() {
//         brakeSolenoid.set(DoubleSolenoid.Value.kForward);
//     }

//     public void disengageBrake() {
//         brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
//     }
// }