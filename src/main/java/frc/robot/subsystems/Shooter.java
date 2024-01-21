// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;


// public class Shooter extends SubsystemBase{
//     CANSparkMax shooter;
//     SparkPIDController shooterPIDController;
//     RelativeEncoder shooterEncoder;
    
//     double gravity = 32.2; // acceleration of gravity, ft/s^2
//     double diameter = Constants.ShooterConstants.SHOOTER_WHEEL_DIAMETER;
//     double angle; //Initial angle ball leaves shooter, could be constant or variable, in degrees
  
//     double kP,kI,kD,kIZ,kFF,kMinOut,kMaxOut;
//     double P,I,D,IZ,FF,MinOut,MaxOut;
//     double setPoint;
//     public Shooter() {
//         shooter = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);

//         kP = Constants.ShooterConstants.SHOOTER_P;
//         kI = Constants.ShooterConstants.SHOOTER_I;
//         kD = Constants.ShooterConstants.SHOOTER_D;
//         kIZ = Constants.ShooterConstants.SHOOTER_IZ;
//         kFF = Constants.ShooterConstants.SHOOTER_FF;
//         kMinOut = Constants.ShooterConstants.SHOOTER_MIN_OUTPUT;
//         kMaxOut = Constants.ShooterConstants.SHOOTER_MAX_OUTPUT;

//         shooterPIDController = shooter.getPIDController();
//         shooterEncoder = shooter.getEncoder();

//         shooterPIDController.setP(kP);
//         shooterPIDController.setI(kI);
//         shooterPIDController.setD(kD);
//         shooterPIDController.setIZone(kIZ);
//         shooterPIDController.setFF(kFF);
//         shooterPIDController.setOutputRange(kMinOut, kMaxOut);

//     }

//     public void setShooterSpeed(double shooterSpeed) {
//         shooter.set(shooterSpeed);
//     }

//     public void setShooterVelocity(double setPoint) {
//         boolean shooterVelocity;
//         SmartDashboard.putNumber("Shooter Setpoint", setPoint);
//         shooterPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
//         SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
//         if(setPoint+50 <= shooterEncoder.getVelocity() && setPoint-50 >= shooterEncoder.getVelocity()) { //Constant may change 
//             shooterVelocity = true;
//         }
//         else {
//             shooterVelocity = false;
//         }
//         SmartDashboard.putBoolean("Shooter is Good", shooterVelocity); //Tells when ready to shoot
//     }

// }