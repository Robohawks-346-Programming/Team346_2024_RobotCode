package frc.robot.subsystems;

import org.photonvision.estimation.RotTrlTransform3d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private static TalonFX pivotMotor;
    private static DoubleSolenoid brakeSolenoid;
    private static TalonFXConfiguration pivotMotorConfig;
    
   private final PositionVoltage position;

   public final InterpolatingDoubleTreeMap pivotLookupTable = Constants.PivotConstants.getPivotMap();
    
    public Pivot() {
        pivotMotor = new TalonFX(Constants.PivotConstants.PIVOT_MOTOR_ID);
        pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivotMotor.setInverted(true);

        pivotMotorConfig.Slot0.kP = Constants.PivotConstants.PIVOT_P;
        pivotMotorConfig.Slot0.kI = Constants.PivotConstants.PIVOT_I;
        pivotMotorConfig.Slot0.kD = Constants.PivotConstants.PIVOT_D;
        pivotMotorConfig.Slot0.kS = Constants.PivotConstants.PIVOT_kS;
        pivotMotorConfig.Slot0.kG = Constants.PivotConstants.PIVOT_kG;

        pivotMotorConfig.Feedback.SensorToMechanismRatio = Constants.PivotConstants.PIVOT_GEAR_RATIO;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PivotConstants.PIVOT_BRAKE_FORWARD_CHANNEL, Constants.PivotConstants.PIVOT_BRAKE_REVERSE_CHANNEL);

        position = new PositionVoltage(0);

        pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", convertRotationsToDegrees(pivotMotor.getPosition().getValue()));

    }

    // Checks to see if the position has been reached
    public boolean isAtPosition(double rev) {
        double difference;
        if (pivotMotor.getPosition().getValueAsDouble() > rev){
            difference = pivotMotor.getPosition().getValueAsDouble() + Math.abs(convertDegreesToRotations(rev));
        } else {
            difference = Math.abs(pivotMotor.getPosition().getValue() - convertDegreesToRotations(rev));
        }
        return(difference <= convertDegreesToRotations(Constants.PivotConstants.PIVOT_ANGLE_THRESHOLD));
    }

    public void moveArmToPosition(double wantedPosition) {
        pivotMotor.setControl(position.withPosition(convertDegreesToRotations(wantedPosition)));
    }

    public void engageBrake() {
        pivotMotor.set(0);
        brakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void disengageBrake() {
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public double convertDegreesToRotations(double degrees){
        return (degrees / 360.0);
    }

    public double convertRotationsToDegrees(double rotations){
        return (rotations * 360.0);
    }

    public void distanceBasePivot() {
        pivotMotor.setControl(position.withPosition(getDistanceBasedAngle()));
    }

    public double getDistanceBasedAngle() {
        return pivotLookupTable.get(RobotContainer.drivetrain.getDistanceFromSpeaker());
    }
    public double armPivot(Pose2d currentDouble){
    double x;
    double y = currentDouble.getY();
    if (DriverStation.getAlliance().get() == Alliance.Blue){
          x = currentDouble.getX();
       } else {
          x = currentDouble.getX();
       }


    
    return (Math.atan2(y,x));
    }
}