// package frc.robot.subsystems;

// import org.photonvision.estimation.RotTrlTransform3d;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
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
//     private static TalonFX rotationMotor;
//     private static DoubleSolenoid brakeSolenoid;
//     private static TalonFXConfiguration pivotMotorConfig;
    
//    private final PositionVoltage anglePosition;

//     double armDegreesPerMotorRev;
    
//     public Pivot() {
//         rotationMotor = new TalonFX(Constants.PivotConstants.PIVOT_MOTOR_ID);
//         pivotMotorConfig = new TalonFXConfiguration();
//         pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         armDegreesPerMotorRev = 360/Constants.PivotConstants.PIVOT_GEAR_RATIO;
        
//         rotationMotor.setInverted(true);

//         pivotMotorConfig.Slot0.kP = Constants.PivotConstants.PIVOT_P;
//         pivotMotorConfig.Slot0.kI = Constants.PivotConstants.PIVOT_I;
//         pivotMotorConfig.Slot0.kD = Constants.PivotConstants.PIVOT_D;

//         rotationMotor.getConfigurator().apply(pivotMotorConfig);

//         brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PivotConstants.PIVOT_BRAKE_FORWARD_CHANNEL, Constants.PivotConstants.PIVOT_BRAKE_REVERSE_CHANNEL);

//         anglePosition = new PositionVoltage(0);

//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Arm Degrees", getRotationEncoder());
//     }

//     // Resetting Rotation encoders
//     public void resetRotationEncoder() {
//         rotationMotor.setPosition(0.0);
//     }

//     public double getRotationEncoder() {
//         return rotationMotor.getPosition().getValue();
//     }

//     public void setRotationEncoder() {
//         rotationMotor.setPosition(Constants.PivotConstants.HOME_PIVOT_ANGLE);
//     }

//     // Checks to see if the position has been reached
//     public boolean isAtPosition(double rev) {
//         double difference = Math.abs(rotationMotor.getPosition().getValue() - rev);
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
//         double currentPosition = rotationMotor.getPosition().getValue();
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
//         rotationMotor.setControl(anglePosition.withPosition(wantedPosition));
//     }

//     public void engageBrake() {
//         brakeSolenoid.set(DoubleSolenoid.Value.kForward);
//     }

//     public void disengageBrake() {
//         brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
//     }
// }