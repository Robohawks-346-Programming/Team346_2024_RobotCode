package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
    private static TalonFX pivotMotor;
    private static TalonFXConfiguration pivotMotorConfig;
    
   private final PositionVoltage position;

   public final InterpolatingDoubleTreeMap pivotLookupTable = Constants.PivotConstants.getPivotMap();

   private double x, y;
    
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

        position = new PositionVoltage(0);

        // pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));
        resetPivotAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", convertRotationsToDegrees(pivotMotor.getPosition().getValue()));

        // if (DriverStation.getAlliance().get() == Alliance.Blue){
        //     x = RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getX() - 0.5;
        // } else {
        //     x = 16 - RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getX();
        // }

        // y = Math.abs(RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getY() - 5.5);
    }
    
    public boolean isAtPosition(double rev) {
        return(Math.abs(pivotMotor.getPosition().getValue() - convertDegreesToRotations(rev)) <= convertDegreesToRotations(Constants.PivotConstants.PIVOT_ANGLE_THRESHOLD));
    }

    public void moveArmToPosition(double wantedPosition) {
        SmartDashboard.putNumber("wanted position", convertDegreesToRotations(wantedPosition));
        pivotMotor.setControl(position.withPosition(convertDegreesToRotations(wantedPosition)));
    }

    public double convertDegreesToRotations(double degrees){
        return (degrees / 360.0);
    }

    public double convertRotationsToDegrees(double rotations){
        return (rotations * 360.0);
    }

    public void distanceBasedArmPivot(){
        pivotMotor.setControl(position.withPosition(getDistanceBasedAngle()));
    }

    public double getDistanceBasedAngle(){
        return pivotLookupTable.get(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }

    public void resetPivotAngle() {
        pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }
}